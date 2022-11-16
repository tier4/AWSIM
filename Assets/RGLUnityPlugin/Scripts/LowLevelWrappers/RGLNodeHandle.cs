using System;

namespace RGLUnityPlugin
{
    public class RGLNodeHandle
    {
        /// <summary>
        /// Allows users to reference node for modifications and prevents users from adding the same node twice by accident.
        /// </summary>
        public string Identifier = "";

        /// <summary>
        /// RGL handle to node (rgl_node_t).
        /// </summary>
        public IntPtr Node = IntPtr.Zero;

        /// <summary>
        /// Allows calling correct API call to modify node's arguments.
        /// </summary>
        public RGLNodeType Type = RGLNodeType.UNKNOWN;
    }
}