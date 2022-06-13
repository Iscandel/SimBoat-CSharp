using DDS;
using System.Runtime.InteropServices;

namespace entity
{
    #region EntityInfo
    [StructLayout(LayoutKind.Sequential)]
    public sealed class EntityInfo
    {
        public short id;
        public string type = string.Empty;
    };
    #endregion

    #region EntityKinematicsInfo
    [StructLayout(LayoutKind.Sequential)]
    public sealed class EntityKinematicsInfo
    {
        public short id;
        public float x;
        public float y;
        public float z;
        public double roll;
        public double pitch;
        public double yaw;
    };
    #endregion

    #region EntityShape
    [StructLayout(LayoutKind.Sequential)]
    public sealed class EntityShape
    {
        public short id;
        public common.Triangle[] triangles_list = new common.Triangle[0];
    };
    #endregion

}

