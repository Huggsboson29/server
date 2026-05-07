/*
===========================================================================

  Copyright (c) 2010-2015 Darkstar Dev Teams

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses/

===========================================================================
*/

#include "navmesh.h"

#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

#include <common/utils.h>
#include <common/xirand.h>

#include <fstream>
#include <set>
#include <vector>

namespace
{

static const int8 ERROR_NEARESTPOLY  = -2;
static const int  NAVMESHSET_MAGIC   = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; // 'MSET'
static const int  NAVMESHSET_VERSION = 1;

// Flags are defined in a "Sample.h" file in the RecastDemo project of recastnavigation.
enum SamplePolyFlags : uint16_t
{
    SAMPLE_POLYFLAGS_WALK     = 0x0001, // Ability to walk (ground, grass, road)
    SAMPLE_POLYFLAGS_SWIM     = 0x0002, // Ability to swim (water).
    SAMPLE_POLYFLAGS_DOOR     = 0x0004, // Ability to move through doors.
    SAMPLE_POLYFLAGS_JUMP     = 0x0008, // Ability to jump.
    SAMPLE_POLYFLAGS_DISABLED = 0x0010, // Disabled polygon
    SAMPLE_POLYFLAGS_ALL      = 0xFFFF, // All abilities.
};

// If a flag is defined in both, then exclude will take priority, and it will be excluded.
// Therefore we can just include all and exclude the ones we don't want.
constexpr unsigned short INCLUDE_FLAGS = SamplePolyFlags::SAMPLE_POLYFLAGS_ALL;
constexpr unsigned short EXCLUDE_FLAGS = SamplePolyFlags::SAMPLE_POLYFLAGS_DISABLED;

// These constants define how large the result buffers for detour are.
// Larger buffers will allow for longer and higher quality paths to be
// found, but will take longer to calculate. Since we use detour in a
// blocking manner, we want to keep these values as low as possible
// while balancing the need for quality paths.
static const size_t MAX_NAV_POLYS     = 512;
static const size_t MAX_HIT_PATH_SIZE = 16;
static const size_t MAX_QUERY_POLYS   = 16;

// These constants define the size of the search extents for
// detour. The larger the extents, the more polys will be searched
// when looking for a path or point. Since we use detour in a blocking
// manner, we want to keep these values as low as possible while
// balancing the need for quality paths.
constexpr float smallPolyPickExt[3]  = { 0.5f, 1.0f, 0.5f };
constexpr float polyPickExt[3]       = { 2.5f, 5.0f, 2.5f };
constexpr float skinnyPolyPickExt[3] = { 0.01f, 10.0f, 0.01f };
constexpr float largePolyPickExt[3]  = { 30.0f, 60.0f, 30.0f };
constexpr float verticalLimit        = 5.0f;

struct NavMeshSetHeader
{
    int             magic;
    int             version;
    int             numTiles;
    dtNavMeshParams params;
};

struct NavMeshTileHeader
{
    dtTileRef tileRef;
    int       dataSize;
};

} // namespace

void CNavMesh::ToFFXIPos(const position_t* pos, float* out)
{
    float y = pos->y;
    float z = pos->z;

    out[0] = pos->x;
    out[1] = y * -1;
    out[2] = z * -1;
}

void CNavMesh::ToFFXIPos(float* out)
{
    float y = out[1];
    float z = out[2];

    out[1] = y * -1;
    out[2] = z * -1;
}

void CNavMesh::ToFFXIPos(position_t* out)
{
    float y = out->y;
    float z = out->z;

    out->y = y * -1;
    out->z = z * -1;
}

void CNavMesh::ToDetourPos(float* out)
{
    float y = out[1];
    float z = out[2];

    out[1] = y * -1;
    out[2] = z * -1;
}

void CNavMesh::ToDetourPos(position_t* out)
{
    float y = out->y;
    float z = out->z;

    out->y = y * -1;
    out->z = z * -1;
}

void CNavMesh::ToDetourPos(const position_t* pos, float* out)
{
    float y = pos->y;
    float z = pos->z;

    out[0] = pos->x;
    out[1] = y * -1;
    out[2] = z * -1;
}

CNavMesh::CNavMesh(uint16 zoneID)
: m_zoneID(zoneID)
, m_navMesh(nullptr)
{
    m_navMeshQueryPolyData.resize(MAX_NAV_POLYS);
    m_navMeshQueryStraightPathFloatData.resize(MAX_NAV_POLYS * 3);
    m_navMeshQueryStraightPathFlagData.resize(MAX_NAV_POLYS);
    m_navMeshQueryStraightPathPolyData.resize(MAX_NAV_POLYS);
}

CNavMesh::~CNavMesh()
{
    if (m_navMesh)
    {
        dtFreeNavMesh(m_navMesh);
    }
}

bool CNavMesh::load(const std::string& filename)
{
    std::ifstream file(filename.c_str(), std::ios_base::in | std::ios_base::binary);

    if (!file.good())
    {
        return false;
    }

    // Read header.
    NavMeshSetHeader header{};
    file.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (header.magic != NAVMESHSET_MAGIC)
    {
        return false;
    }
    if (header.version != NAVMESHSET_VERSION)
    {
        return false;
    }

    m_navMesh = dtAllocNavMesh();
    if (!m_navMesh)
    {
        return false;
    }

    dtStatus status = m_navMesh->init(&header.params);
    if (dtStatusFailed(status))
    {
        ShowError("CNavMesh::load Could not initialize detour for (%s)", filename);
        ShowError(detourStatusString(status));
        return false;
    }

    // Read tiles.
    for (int i = 0; i < header.numTiles; ++i)
    {
        NavMeshTileHeader tileHeader{};
        file.read(reinterpret_cast<char*>(&tileHeader), sizeof(tileHeader));
        if (!tileHeader.tileRef || !tileHeader.dataSize)
        {
            break;
        }

        unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
        if (!data)
        {
            break;
        }
        std::memset(data, 0, tileHeader.dataSize);
        file.read(reinterpret_cast<char*>(data), tileHeader.dataSize);

        m_navMesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, nullptr);
    }

    // init detour nav mesh path finder
    status = m_navMeshQuery.init(m_navMesh, MAX_NAV_POLYS);

    if (dtStatusFailed(status))
    {
        ShowError("CNavMesh::load Error loading m_navMeshQuery (%s)", filename);
        ShowError(detourStatusString(status));
        return false;
    }

    return true;
}

void CNavMesh::unload()
{
    dtFreeNavMesh(m_navMesh);
    m_navMesh = nullptr;
}

bool CNavMesh::installNavMesh(dtNavMesh* newNavMesh)
{
    if (!newNavMesh)
    {
        return false;
    }

    unload();

    m_navMesh = newNavMesh;

    const auto status = m_navMeshQuery.init(m_navMesh, MAX_NAV_POLYS);
    if (dtStatusFailed(status))
    {
        ShowErrorFmt("CNavMesh::installNavMesh: Could not init navMeshQuery ({})", m_zoneID);
        unload();
        return false;
    }

    return true;
}

bool CNavMesh::save(const std::string& path) const
{
    if (!m_navMesh || path.empty())
    {
        return false;
    }

    std::ofstream file(path, std::ios::binary);
    if (!file.good())
    {
        ShowErrorFmt("CNavMesh::save: Could not open file for writing ({})", path);
        return false;
    }

    const auto* nav = m_navMesh;

    auto header    = NavMeshSetHeader{};
    header.magic   = NAVMESHSET_MAGIC;
    header.version = NAVMESHSET_VERSION;
    header.params  = *nav->getParams();

    for (auto i = 0; i < nav->getMaxTiles(); ++i)
    {
        const auto* tile = nav->getTile(i);
        if (tile && tile->header && tile->dataSize > 0)
        {
            header.numTiles++;
        }
    }

    file.write(reinterpret_cast<const char*>(&header), sizeof(header));

    for (auto i = 0; i < nav->getMaxTiles(); ++i)
    {
        const auto* tile = nav->getTile(i);
        if (!tile || !tile->header || tile->dataSize <= 0)
        {
            continue;
        }

        const auto tileHeader = NavMeshTileHeader{
            .tileRef  = nav->getTileRef(tile),
            .dataSize = tile->dataSize,
        };

        file.write(reinterpret_cast<const char*>(&tileHeader), sizeof(tileHeader));
        file.write(reinterpret_cast<const char*>(tile->data), tile->dataSize);
    }

    return true;
}

auto CNavMesh::findPath(const position_t& start, const position_t& end) -> std::vector<pathpoint_t>
{
    TracyZoneScoped;

    if (std::isnan(start.x) || std::isnan(start.y) || std::isnan(start.z) ||
        std::isnan(end.x) || std::isnan(end.y) || std::isnan(end.z))
    {
        ShowWarning("CNavMesh::findPath NaN position detected (%u)", m_zoneID);
        return {};
    }

    DebugNavmesh("CNavMesh::findPath (%f, %f, %f) -> (%f, %f, %f) (zone: %u) (MAX_NAV_POLYS: %u)", start.x, start.y, start.z, end.x, end.y, end.z, m_zoneID, MAX_NAV_POLYS);
    dtStatus status = 0;

    float spos[3];
    CNavMesh::ToDetourPos(&start, spos);

    float epos[3];
    CNavMesh::ToDetourPos(&end, epos);

    dtQueryFilter filter;
    filter.setIncludeFlags(INCLUDE_FLAGS);
    filter.setExcludeFlags(EXCLUDE_FLAGS);

    dtPolyRef startRef = 0;
    dtPolyRef endRef   = 0;

    float sNearestPoint[3];
    float eNearestPoint[3];

    // Validate spos into startRef and sNearestPoint
    status = m_navMeshQuery.findNearestPoly(spos, polyPickExt, &filter, &startRef, sNearestPoint);
    if (dtStatusFailed(status))
    {
        DebugNavmesh("CNavMesh::findPath start point invalid (%f, %f, %f) (%u)", spos[0], spos[1], spos[2], m_zoneID);
        ShowError(detourStatusString(status));
        return {};
    }

    // Validate epos into endRef and eNearestPoint
    status = m_navMeshQuery.findNearestPoly(epos, polyPickExt, &filter, &endRef, eNearestPoint);
    if (dtStatusFailed(status))
    {
        ShowError("CNavMesh::findPath end point invalid (%f, %f, %f) (%u)", epos[0], epos[1], epos[2], m_zoneID);
        ShowError(detourStatusString(status));
        return {};
    }

    // TODO: Do we need these isValidPolyRef checks? We've just found the nearest polys and checked them with dtStatusFailed.

    // Make sure the start poly is valid
    if (!m_navMesh->isValidPolyRef(startRef))
    {
        DebugNavmesh("CNavMesh::findPath Start poly invalid: (%f, %f, %f) (%u)", start.x, start.y, start.z, m_zoneID);
        return {};
    }

    // Make sure the end poly is valid
    if (!m_navMesh->isValidPolyRef(endRef))
    {
        DebugNavmesh("CNavMesh::findPath End poly invalid: (%f, %f, %f) (%u)", end.x, end.y, end.z, m_zoneID);
        return {};
    }

    // First, we're going to build up a list of polys that make up the path
    int32 pathPolyCount = 0;

    status = m_navMeshQuery.findPath(startRef, endRef, sNearestPoint, eNearestPoint, &filter, m_navMeshQueryPolyData.data(), &pathPolyCount, MAX_NAV_POLYS);
    if (dtStatusFailed(status))
    {
        ShowError("CNavMesh::findPath findPath error (%u)", m_zoneID);
        ShowError(detourStatusString(status));
        return {};
    }

    // At this point, findPath() has generated a list of polys that make up a path, but these polys aren't guaranteed to contain a complete path
    // between the start and end points we've requested. When we call findStraightPath() later, it will generate the best path between
    // sNearestPoint and eNearestPoint that it can within the bounds of the polys we've found here.

    if (pathPolyCount <= 0)
    {
        ShowError("CNavMesh::findPath Unable to generate polys for path (%f, %f, %f)->(%f, %f, %f) (%u)", start.x, start.y, start.z, end.x, end.y, end.z, m_zoneID);
        return {};
    }

    // Find the best straight path possible between sNearestPoint and eNearestPoint within the bounds of all the polys in pathPolys.
    int32 straightPathCount = 0;

    // NOTE: The DT_STRAIGHTPATH_ALL_CROSSINGS flag can exasorbate the issue of getting trapped in local minima.
    status = m_navMeshQuery.findStraightPath(sNearestPoint, eNearestPoint, m_navMeshQueryPolyData.data(), pathPolyCount, m_navMeshQueryStraightPathFloatData.data(), m_navMeshQueryStraightPathFlagData.data(), m_navMeshQueryStraightPathPolyData.data(), &straightPathCount, MAX_NAV_POLYS /*, DT_STRAIGHTPATH_ALL_CROSSINGS */);

    if (dtStatusFailed(status))
    {
        ShowError("CNavMesh::findPath findStraightPath error (%u)", m_zoneID);
        ShowError(detourStatusString(status));
        return {};
    }

    // Now that we have list of sequential positions in straightPath, we need to check to see if eNearestPoint is the final point. If it isn't we've been given a partial path.
    // If we have a partial path, the final point is a best-effort attempt by Detour to get us as close to eNearestPoint as possible. This can end up trapping us in
    // local minima (ie. corners we can't navigate out of). Therefore, if we have a partial path we're going to omit the final point.
    const auto eNearestPosition = position_t{
        eNearestPoint[0],
        eNearestPoint[1],
        eNearestPoint[2],
        0,
        0,
    };

    const auto pathEndPosition = position_t{
        m_navMeshQueryStraightPathFloatData[straightPathCount * 3 - 3],
        m_navMeshQueryStraightPathFloatData[straightPathCount * 3 - 2],
        m_navMeshQueryStraightPathFloatData[straightPathCount * 3 - 1],
        0,
        0,
    };

    // bool partialPath = false;
    if (straightPathCount > 2 && distance(eNearestPosition, pathEndPosition) > 5.0f)
    {
        DebugNavmesh("CNavMesh::findPath Partial path detected! (%u)", m_zoneID);
        // partialPath = true;
        straightPathCount -= 1;
    }

    // If there is only a single point, that is our starting point.
    // We have now exhausted our path, return empty results.
    if (straightPathCount <= 1)
    {
        return {};
    }

    // TODO: Detect local minima and re-try pathing with a larger buffer.

    // NOTE: i starts at 3 so the start position is ignored
    std::vector<pathpoint_t> outPoints;
    outPoints.reserve(straightPathCount - 1);
    for (int i = 3; i < straightPathCount * 3;)
    {
        float pathPos[3];
        pathPos[0] = m_navMeshQueryStraightPathFloatData[i++];
        pathPos[1] = m_navMeshQueryStraightPathFloatData[i++];
        pathPos[2] = m_navMeshQueryStraightPathFloatData[i++];

        CNavMesh::ToFFXIPos(pathPos);

        outPoints.emplace_back(pathpoint_t{ { pathPos[0], pathPos[1], pathPos[2], 0, 0 }, 0s, false });
    }

    return outPoints;
}

std::pair<int16, position_t> CNavMesh::findRandomPosition(const position_t& start, float maxRadius)
{
    TracyZoneScoped;

    DebugNavmesh("CNavMesh::findRandomPosition (%f, %f, %f) (%u)", start.x, start.y, start.z, m_zoneID);

    dtStatus status = 0;

    float spos[3];
    CNavMesh::ToDetourPos(&start, spos);

    float randomPt[3];
    float snearest[3];

    dtQueryFilter filter;
    filter.setIncludeFlags(INCLUDE_FLAGS);
    filter.setExcludeFlags(EXCLUDE_FLAGS);

    dtPolyRef startRef  = 0;
    dtPolyRef randomRef = 0;

    status = m_navMeshQuery.findNearestPoly(spos, polyPickExt, &filter, &startRef, snearest);

    if (dtStatusFailed(status))
    {
        ShowError("CNavMesh::findRandomPosition start point invalid (%f, %f, %f) (%u)", spos[0], spos[1], spos[2], m_zoneID);
        ShowError(detourStatusString(status));
        return std::make_pair(ERROR_NEARESTPOLY, position_t{});
    }

    if (!m_navMesh->isValidPolyRef(startRef))
    {
        DebugNavmesh("CNavMesh::findRandomPosition startRef is invalid (%f, %f, %f) (%u)", start.x, start.y, start.z, m_zoneID);
        return std::make_pair(ERROR_NEARESTPOLY, position_t{});
    }

    status = m_navMeshQuery.findRandomPointAroundCircle(
        startRef,
        spos,
        maxRadius,
        &filter,
        []() -> float
        {
            return xirand::GetRandomNumber(1.0f);
        },
        &randomRef,
        randomPt);

    if (dtStatusFailed(status))
    {
        ShowError("CNavMesh::findRandomPosition Error (%u)", m_zoneID);
        ShowError(detourStatusString(status));
        return std::make_pair(ERROR_NEARESTPOLY, position_t{});
    }

    CNavMesh::ToFFXIPos(randomPt);

    return std::make_pair(0, position_t{ randomPt[0], randomPt[1], randomPt[2], 0, 0 });
}

bool CNavMesh::validPosition(const position_t& position)
{
    TracyZoneScoped;

    DebugNavmesh("CNavMesh::validPosition (%f, %f, %f) (%u)", position.x, position.y, position.z, m_zoneID);

    float spos[3];
    CNavMesh::ToDetourPos(&position, spos);

    float snearest[3];

    dtQueryFilter filter;
    filter.setIncludeFlags(INCLUDE_FLAGS);
    filter.setExcludeFlags(EXCLUDE_FLAGS);

    dtPolyRef startRef = 0;

    dtStatus status = m_navMeshQuery.findNearestPoly(spos, smallPolyPickExt, &filter, &startRef, snearest);

    if (dtStatusFailed(status))
    {
        return false;
    }

    return m_navMesh->isValidPolyRef(startRef);
}

bool CNavMesh::findClosestValidPoint(const position_t& position, float* validPoint)
{
    TracyZoneScoped;

    DebugNavmesh("CNavMesh::findClosestValidPoint (%f, %f, %f) (%u)", position.x, position.y, position.z, m_zoneID);

    float spos[3];
    CNavMesh::ToDetourPos(&position, spos);

    dtQueryFilter filter;
    filter.setIncludeFlags(INCLUDE_FLAGS);
    filter.setExcludeFlags(EXCLUDE_FLAGS);

    dtPolyRef startRef = 0;

    dtStatus status = m_navMeshQuery.findNearestPoly(spos, largePolyPickExt, &filter, &startRef, validPoint);

    if (dtStatusFailed(status))
    {
        return false;
    }

    CNavMesh::ToFFXIPos(validPoint);
    return true;
}

bool CNavMesh::findFurthestValidPoint(const position_t& startPosition, const position_t& endPosition, float* validEndPoint)
{
    TracyZoneScoped;

    DebugNavmesh("CNavMesh::findFurthestValidPoint (%f, %f, %f) -> (%f, %f, %f) (%u)", startPosition.x, startPosition.y, startPosition.z, endPosition.x, endPosition.y, endPosition.z, m_zoneID);

    float spos[3];
    CNavMesh::ToDetourPos(&startPosition, spos);

    dtQueryFilter filter;
    filter.setIncludeFlags(INCLUDE_FLAGS);
    filter.setExcludeFlags(EXCLUDE_FLAGS);

    dtPolyRef startRef = 0;
    float     validStartPoint[3];

    dtStatus status = m_navMeshQuery.findNearestPoly(spos, largePolyPickExt, &filter, &startRef, validStartPoint);
    if (dtStatusFailed(status))
    {
        return false;
    }

    dtPolyRef visited[MAX_QUERY_POLYS];
    int       visitedCount = 0;

    float targetPoint[3];
    CNavMesh::ToDetourPos(&endPosition, targetPoint);

    status = m_navMeshQuery.moveAlongSurface(startRef, validStartPoint, targetPoint, &filter, validEndPoint, visited, &visitedCount, MAX_QUERY_POLYS);

    if (dtStatusFailed(status))
    {
        return false;
    }

    CNavMesh::ToFFXIPos(validEndPoint);
    return true;
}

void CNavMesh::snapToValidPosition(position_t& position)
{
    TracyZoneScoped;

    DebugNavmesh("CNavMesh::snapToValidPosition (%f, %f, %f) (%u)", position.x, position.y, position.z, m_zoneID);

    float spos[3];
    CNavMesh::ToDetourPos(&position, spos);

    float snearest[3];

    dtQueryFilter filter;
    filter.setIncludeFlags(INCLUDE_FLAGS);
    filter.setExcludeFlags(EXCLUDE_FLAGS);

    dtPolyRef startRef = 0;

    dtStatus status = m_navMeshQuery.findNearestPoly(spos, polyPickExt, &filter, &startRef, snearest);

    if (dtStatusFailed(status))
    {
        ShowError("CNavMesh::Failed to find nearby valid poly (%f, %f, %f) (%u)", spos[0], spos[1], spos[2], m_zoneID);
        ShowError(detourStatusString(status));
        return;
    }

    if (m_navMesh->isValidPolyRef(startRef))
    {
        CNavMesh::ToFFXIPos(snearest);
        position.x = snearest[0];
        position.y = snearest[1];
        position.z = snearest[2];
    }
}

bool CNavMesh::moveAlongSurface(const position_t& start, const position_t& end, position_t& result)
{
    TracyZoneScoped;

    float spos[3];
    CNavMesh::ToDetourPos(&start, spos);

    float epos[3];
    CNavMesh::ToDetourPos(&end, epos);

    dtQueryFilter filter;
    filter.setIncludeFlags(INCLUDE_FLAGS);
    filter.setExcludeFlags(EXCLUDE_FLAGS);

    dtPolyRef startRef = 0;
    float     snearest[3];

    dtStatus status = m_navMeshQuery.findNearestPoly(spos, polyPickExt, &filter, &startRef, snearest);
    if (dtStatusFailed(status) || !m_navMesh->isValidPolyRef(startRef))
    {
        return false;
    }

    float     resultPos[3];
    dtPolyRef visited[MAX_QUERY_POLYS];
    int       visitedCount = 0;

    status = m_navMeshQuery.moveAlongSurface(
        startRef,
        snearest,
        epos,
        &filter,
        resultPos,
        visited,
        &visitedCount,
        MAX_QUERY_POLYS);

    if (dtStatusFailed(status))
    {
        return false;
    }

    CNavMesh::ToFFXIPos(resultPos);
    result.x = resultPos[0];
    result.y = resultPos[1];
    result.z = resultPos[2];

    return true;
}

bool CNavMesh::moveAlongSurface(unsigned int& cachedRef, const position_t& start, const position_t& end, position_t& result)
{
    TracyZoneScoped;

    float spos[3];
    CNavMesh::ToDetourPos(&start, spos);

    float epos[3];
    CNavMesh::ToDetourPos(&end, epos);

    dtQueryFilter filter;
    filter.setIncludeFlags(INCLUDE_FLAGS);
    filter.setExcludeFlags(EXCLUDE_FLAGS);

    dtPolyRef startRef = 0;
    float     snearest[3];

    if (cachedRef && m_navMesh->isValidPolyRef((dtPolyRef)cachedRef))
    {
        bool     posOverPoly = false;
        dtStatus status      = m_navMeshQuery.closestPointOnPoly((dtPolyRef)cachedRef, spos, snearest, &posOverPoly);
        if (dtStatusSucceed(status) && posOverPoly)
        {
            startRef = (dtPolyRef)cachedRef;
        }
    }

    if (!startRef)
    {
        dtStatus status = m_navMeshQuery.findNearestPoly(spos, polyPickExt, &filter, &startRef, snearest);
        if (dtStatusFailed(status) || !m_navMesh->isValidPolyRef(startRef))
        {
            return false;
        }
    }

    float     resultPos[3];
    dtPolyRef visited[MAX_QUERY_POLYS];
    int       visitedCount = 0;

    dtStatus status = m_navMeshQuery.moveAlongSurface(
        startRef,
        snearest,
        epos,
        &filter,
        resultPos,
        visited,
        &visitedCount,
        MAX_QUERY_POLYS);

    if (dtStatusFailed(status))
    {
        return false;
    }

    if (visitedCount > 0)
    {
        cachedRef = visited[visitedCount - 1];
    }

    CNavMesh::ToFFXIPos(resultPos);
    result.x = resultPos[0];
    result.y = resultPos[1];
    result.z = resultPos[2];

    return true;
}

[[nodiscard]] auto CNavMesh::detourStatusString(const uint32 status) -> std::string
{
    std::string outStr;

    // High level status.
    if (status & DT_FAILURE)
    {
        outStr += "DT_FAILURE: Operation failed. ";
    }
    if (status & DT_SUCCESS)
    {
        outStr += "DT_SUCCESS: Operation succeeded. ";
    }
    if (status & DT_IN_PROGRESS)
    {
        outStr += "DT_IN_PROGRESS: Operation still in progress. ";
    }

    // Detail information for status.
    if (status & DT_WRONG_MAGIC)
    {
        outStr += "DT_WRONG_MAGIC: Input data is not recognized. ";
    }
    if (status & DT_WRONG_VERSION)
    {
        outStr += "DT_WRONG_VERSION: Input data is in wrong version. ";
    }
    if (status & DT_OUT_OF_MEMORY)
    {
        outStr += "DT_OUT_OF_MEMORY: Operation ran out of memory. ";
    }
    if (status & DT_INVALID_PARAM)
    {
        outStr += "DT_INVALID_PARAM: An input parameter was invalid. ";
    }
    if (status & DT_BUFFER_TOO_SMALL)
    {
        outStr += "DT_BUFFER_TOO_SMALL: Result buffer for the query was too small to store all results. ";
    }
    if (status & DT_OUT_OF_NODES)
    {
        outStr += "DT_OUT_OF_NODES: Query ran out of nodes during search. ";
    }
    if (status & DT_PARTIAL_RESULT)
    {
        outStr += "DT_PARTIAL_RESULT: Query did not reach the end location, returning best guess. ";
    }
    if (status & DT_ALREADY_OCCUPIED)
    {
        outStr += "DT_ALREADY_OCCUPIED: A tile has already been assigned to the given x, y coordinate. ";
    }

    return outStr;
}
