using System;

namespace RGLUnityPlugin
{
    public class RGLException : Exception
    {
        public RGLException()
        {
        }

        public RGLException(string message)
            : base(message)
        {
        }

        public RGLException(string message, Exception inner)
            : base(message, inner)
        {
        }
    }
}