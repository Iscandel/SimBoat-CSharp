using DDS;
using System.Runtime.InteropServices;

namespace common
{
    #region Vector3
    [StructLayout(LayoutKind.Sequential)]
    public sealed class Vector3
    {
        public double x;
        public double y;
        public double z;
    };
    #endregion

    #region Triangle
    [StructLayout(LayoutKind.Sequential)]
    public sealed class Triangle
    {
        public common.Vector3 p1 = new common.Vector3();
        public common.Vector3 p2 = new common.Vector3();
        public common.Vector3 p3 = new common.Vector3();
    };
    #endregion

}

