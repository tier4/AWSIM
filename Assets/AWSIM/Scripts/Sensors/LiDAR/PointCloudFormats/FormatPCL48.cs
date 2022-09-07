using System.Collections.Generic;
using RGLUnityPlugin;
using sensor_msgs.msg;
using std_msgs.msg;

namespace AWSIM.PointCloudFormats
{ 
    public static class FormatPCL48
    {
        public static RGLField[] GetRGLFields()
        {
            return new[]
            {
                RGLField.XYZ_F32,
                RGLField.PADDING_32,
                RGLField.INTENSITY_F32,
                RGLField.RING_ID_U16,
                RGLField.AZIMUTH_F32,
                RGLField.DISTANCE_F32,
                RGLField.RETURN_TYPE_U8,
                RGLField.TIME_STAMP_F64
            };
        }

        public static PointCloud2 GetSensorMsg()
        {
            return new PointCloud2
            {
                Header = new Header(), // TO BE FILLED 
                Data = null, // TO BE FILLED
                Is_bigendian = false,
                Width = 0, // TO BE FILLED
                Height = 1,
                Is_dense = true,
                Point_step = 48,
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
                    },
                    new PointField
                    {
                        Name = "azimuth",
                        Count = 1,
                        Datatype = PointField.FLOAT32,
                        Offset = 24,
                    },
                    new PointField
                    {
                        Name = "distance",
                        Count = 1,
                        Datatype = PointField.FLOAT32,
                        Offset = 28,
                    },
                    new PointField
                    {
                        Name = "return_type",
                        Count = 1,
                        Datatype = PointField.UINT8,
                        Offset = 32
                    },
                    new PointField
                    {
                        Name = "time_stamp",
                        Count = 1,
                        Datatype = PointField.FLOAT64,
                        Offset = 40,
                    }
                }
            };
        }
    }
}