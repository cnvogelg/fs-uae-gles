#ifndef FS_GLU_H_
#define FS_GLU_H_

#if defined(USE_GLES) && !defined(FAKE_GLES)
// no GLU
#elif defined(MACOSX)
#include <OpenGL/glu.h>
#elif defined(USE_GLES)
#include <GLES/glu.h>
#else
#include <GL/glu.h>
#endif

#endif // FS_GLU_H_
