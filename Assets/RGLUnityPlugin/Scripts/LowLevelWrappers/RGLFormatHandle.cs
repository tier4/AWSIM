using System;

namespace RGLUnityPlugin
{
    public class RGLOutputHandle
    {
        public IntPtr node = IntPtr.Zero; // rgl_node_t
        public RGLField field = 0; // rgl_field_t
    }
}