using System;

namespace RGLUnityPlugin
{
    public class RGLNodeHandle
    {
        public string Identifier = "";
        public IntPtr Node = IntPtr.Zero; // rgl_node_t
        public RGLNodeType Type = RGLNodeType.UNKNOWN;
    }
}