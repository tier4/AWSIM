using System;

namespace RGLUnityPlugin
{
    public class RGLNodeHandle
    {
        public IntPtr node = IntPtr.Zero; // rgl_node_t
        private RGLNodeType type = RGLNodeType.UNKNOWN;

        public RGLNodeHandle(RGLNodeType nodeType)
        {
            type = nodeType;
        }

        public RGLNodeType GetNodeType()
        {
            return type;
        }
    }
}