#ifndef FS_GLU_H_
#define FS_GLU_H_

#if defined(USE_GLES) && !defined(FAKE_GLES)
// not used for GLES
#elif defined(MACOSX)
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#endif // FS_GLU_H_
