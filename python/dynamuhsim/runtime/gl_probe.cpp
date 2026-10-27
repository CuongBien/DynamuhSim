// Query a real, unmapped GLX context on the same WSLg display as Gazebo.
#include <GL/gl.h>
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <iomanip>
#include <iostream>
int main() {
  Display *d=XOpenDisplay(nullptr);
  if (!d) {std::cerr << "Cannot open DISPLAY\n";return 2;}
  int attrs[]={GLX_X_RENDERABLE,True,GLX_DRAWABLE_TYPE,GLX_PBUFFER_BIT,GLX_RENDER_TYPE,GLX_RGBA_BIT,None};
  int n=0;GLXFBConfig *fb=glXChooseFBConfig(d,DefaultScreen(d),attrs,&n);
  if (!fb || n==0) {std::cerr << "No GLX pbuffer config\n";return 3;}
  GLXContext c=glXCreateNewContext(d,fb[0],GLX_RGBA_TYPE,nullptr,True);
  int pa[]={GLX_PBUFFER_WIDTH,1,GLX_PBUFFER_HEIGHT,1,None};
  GLXPbuffer p=glXCreatePbuffer(d,fb[0],pa);XFree(fb);
  if (!c || !p || !glXMakeContextCurrent(d,p,p,c)) {std::cerr << "Cannot create current GLX context\n";return 4;}
  auto value=[](GLenum e){auto s=glGetString(e);return s?reinterpret_cast<const char *>(s):"unavailable";};
  glClearColor(0,0,0,1);glClear(GL_COLOR_BUFFER_BIT);glFinish();
  std::cout << "{\"vendor\":" << std::quoted(value(GL_VENDOR))
            << ",\"renderer\":" << std::quoted(value(GL_RENDERER))
            << ",\"version\":" << std::quoted(value(GL_VERSION))
            << ",\"direct\":" << (glXIsDirect(d,c)?"true":"false") << "}\n";
  glXMakeContextCurrent(d,None,None,nullptr);glXDestroyPbuffer(d,p);glXDestroyContext(d,c);XCloseDisplay(d);
}
