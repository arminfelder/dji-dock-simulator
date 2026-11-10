# Owned by the OSM port: Overpass, cells, footprints, roofs, facades, grid, provider.
target_sources(docksim PRIVATE
    src/osm/cells.cpp
    src/osm/facades.cpp
    src/osm/features.cpp
    src/osm/footprints.cpp
    src/osm/grid.cpp
    src/osm/overpass.cpp
    src/osm/provider.cpp
    src/osm/roofs.cpp)
