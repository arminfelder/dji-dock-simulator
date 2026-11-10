# Owned by the geodata port: tiles, DEM, imagery, ALS.
target_sources(docksim PRIVATE
    src/raster/als.cpp
    src/render/geodata/imagery.cpp
    src/render/geodata/terrain.cpp
    src/render/geodata/tiles.cpp)
