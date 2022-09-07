using System.Collections.Generic;
using RGLUnityPlugin;
using sensor_msgs.msg;

namespace AWSIM.PointCloudFormats
{
    public static class FormatPCL24
    { 
        public static RGLField[] GetRGLFields()
        {
            return new[]
            {
                RGLField.XYZ_F32,
                RGLField.PADDING_32,
                RGLField.INTENSITY_F32,
                RGLField.RING_ID_U16
            };
        }
        
        public static PointCloud2 GetSensorMsg()
        {
            return new PointCloud2()
            {
                Header = new std_msgs.msg.Header(), // TO BE FILLED 
                Data = null, // TO BE FILLED
                Is_bigendian = false,
                Width = 0, // TO BE FILLED
                Height = 1,
                Is_dense = true,
                Point_step = 24,
                Row_step = 0, // TO BE FILLED,
                Fields = new[]
                {
                    new PointField
                    {
                        Name = "x",
                        Count = 1,
                        Datatype = PointField.FLOAT32,
                        Offset = 0,
                    },
                    new PointField
                    {
                        Name = "y",
                        Count = 1,
                        Datatype = PointField.FLOAT32,
                        Offset = 4,
                    },
                    new PointField
                    {
                        Name = "z",
                        Count = 1,
                        Datatype = PointField.FLOAT32,
                        Offset = 8,
                    },
                    new PointField
                    {
                        Name = "intensity",
                        Count = 1,
                        Datatype = PointField.FLOAT32,
                        Offset = 16,
                    },
                    new PointField
                    {
                        Name = "ring",
                        Count = 1,
                        Datatype = PointField.UINT16,
                        Offset = 20,
                    }
                }
            };
        }
    }
}