// The one translation unit that holds the stb implementations.
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_TRUETYPE_IMPLEMENTATION

#include <stb_image.h>
#include <stb_image_write.h>
#include <stb_truetype.h>
