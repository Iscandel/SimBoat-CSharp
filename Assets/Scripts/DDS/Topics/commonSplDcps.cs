using DDS;
using DDS.OpenSplice.CustomMarshalers;
using DDS.OpenSplice.Database;
using DDS.OpenSplice.Kernel;
using System;
using System.Runtime.InteropServices;

namespace common
{
    #region __Vector3
    [StructLayout(LayoutKind.Sequential)]
    public struct __Vector3
    {
        public double x;
        public double y;
        public double z;
    }
    #endregion

    #region __Vector3Marshaler
    public sealed class __Vector3Marshaler : DDS.OpenSplice.CustomMarshalers.FooDatabaseMarshaler<common.Vector3>
    {
        public static readonly string fullyScopedName = "common::Vector3";

        public override void InitEmbeddedMarshalers(IDomainParticipant participant)
        {
        }

        public override V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, System.IntPtr from, System.IntPtr to)
        {
            GCHandle tmpGCHandle = GCHandle.FromIntPtr(from);
            common.Vector3 fromData = tmpGCHandle.Target as common.Vector3;
            return CopyIn(typePtr, fromData, to);
        }

        public V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, common.Vector3 from, System.IntPtr to)
        {
            __Vector3 nativeImg = new __Vector3();
            V_COPYIN_RESULT result = CopyIn(typePtr, from, ref nativeImg);
            if (result == V_COPYIN_RESULT.OK)
            {
                Marshal.StructureToPtr(nativeImg, to, false);
            }
            return result;
        }

        public V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, common.Vector3 from, ref __Vector3 to)
        {
            if (from == null) return V_COPYIN_RESULT.INVALID;
            to.x = from.x;
            to.y = from.y;
            to.z = from.z;
            return V_COPYIN_RESULT.OK;
        }

        public override void CopyOut(System.IntPtr from, System.IntPtr to)
        {
            __Vector3 nativeImg = (__Vector3) Marshal.PtrToStructure(from, typeof(__Vector3));
            GCHandle tmpGCHandleTo = GCHandle.FromIntPtr(to);
            common.Vector3 toObj = tmpGCHandleTo.Target as common.Vector3;
            CopyOut(ref nativeImg, ref toObj);
            tmpGCHandleTo.Target = toObj;
        }

        public override void CopyOut(System.IntPtr from, ref common.Vector3 to)
        {
            __Vector3 nativeImg = (__Vector3) Marshal.PtrToStructure(from, typeof(__Vector3));
            CopyOut(ref nativeImg, ref to);
        }

        public static void StaticCopyOut(System.IntPtr from, ref common.Vector3 to)
        {
            __Vector3 nativeImg = (__Vector3) Marshal.PtrToStructure(from, typeof(__Vector3));
            CopyOut(ref nativeImg, ref to);
        }

        public static void CopyOut(ref __Vector3 from, ref common.Vector3 to)
        {
            if (to == null) {
                to = new common.Vector3();
            }
            to.x = from.x;
            to.y = from.y;
            to.z = from.z;
        }

    }
    #endregion

    #region __Triangle
    [StructLayout(LayoutKind.Sequential)]
    public struct __Triangle
    {
        public common.__Vector3 p1;
        public common.__Vector3 p2;
        public common.__Vector3 p3;
    }
    #endregion

    #region __TriangleMarshaler
    public sealed class __TriangleMarshaler : DDS.OpenSplice.CustomMarshalers.FooDatabaseMarshaler<common.Triangle>
    {
        public static readonly string fullyScopedName = "common::Triangle";
        private common.__Vector3Marshaler attr0Marshaler;
        private common.__Vector3Marshaler attr1Marshaler;
        private common.__Vector3Marshaler attr2Marshaler;

        public override void InitEmbeddedMarshalers(IDomainParticipant participant)
        {
            if (attr0Marshaler == null) {
                attr0Marshaler = DatabaseMarshaler.GetMarshaler(participant, typeof(common.__Vector3)) as common.__Vector3Marshaler;
                if (attr0Marshaler == null) {
                    attr0Marshaler = new common.__Vector3Marshaler();
                    DatabaseMarshaler.Add(participant, typeof(common.__Vector3), attr0Marshaler);
                    attr0Marshaler.InitEmbeddedMarshalers(participant);
                }
            }
            if (attr1Marshaler == null) {
                attr1Marshaler = DatabaseMarshaler.GetMarshaler(participant, typeof(common.__Vector3)) as common.__Vector3Marshaler;
                if (attr1Marshaler == null) {
                    attr1Marshaler = new common.__Vector3Marshaler();
                    DatabaseMarshaler.Add(participant, typeof(common.__Vector3), attr1Marshaler);
                    attr1Marshaler.InitEmbeddedMarshalers(participant);
                }
            }
            if (attr2Marshaler == null) {
                attr2Marshaler = DatabaseMarshaler.GetMarshaler(participant, typeof(common.__Vector3)) as common.__Vector3Marshaler;
                if (attr2Marshaler == null) {
                    attr2Marshaler = new common.__Vector3Marshaler();
                    DatabaseMarshaler.Add(participant, typeof(common.__Vector3), attr2Marshaler);
                    attr2Marshaler.InitEmbeddedMarshalers(participant);
                }
            }
        }

        public override V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, System.IntPtr from, System.IntPtr to)
        {
            GCHandle tmpGCHandle = GCHandle.FromIntPtr(from);
            common.Triangle fromData = tmpGCHandle.Target as common.Triangle;
            return CopyIn(typePtr, fromData, to);
        }

        public V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, common.Triangle from, System.IntPtr to)
        {
            __Triangle nativeImg = new __Triangle();
            V_COPYIN_RESULT result = CopyIn(typePtr, from, ref nativeImg);
            if (result == V_COPYIN_RESULT.OK)
            {
                Marshal.StructureToPtr(nativeImg, to, false);
            }
            return result;
        }

        public V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, common.Triangle from, ref __Triangle to)
        {
            if (from == null) return V_COPYIN_RESULT.INVALID;
            {
                V_COPYIN_RESULT result = attr0Marshaler.CopyIn(typePtr, from.p1, ref to.p1);
                if (result != V_COPYIN_RESULT.OK) return result;
            }
            {
                V_COPYIN_RESULT result = attr1Marshaler.CopyIn(typePtr, from.p2, ref to.p2);
                if (result != V_COPYIN_RESULT.OK) return result;
            }
            {
                V_COPYIN_RESULT result = attr2Marshaler.CopyIn(typePtr, from.p3, ref to.p3);
                if (result != V_COPYIN_RESULT.OK) return result;
            }
            return V_COPYIN_RESULT.OK;
        }

        public override void CopyOut(System.IntPtr from, System.IntPtr to)
        {
            __Triangle nativeImg = (__Triangle) Marshal.PtrToStructure(from, typeof(__Triangle));
            GCHandle tmpGCHandleTo = GCHandle.FromIntPtr(to);
            common.Triangle toObj = tmpGCHandleTo.Target as common.Triangle;
            CopyOut(ref nativeImg, ref toObj);
            tmpGCHandleTo.Target = toObj;
        }

        public override void CopyOut(System.IntPtr from, ref common.Triangle to)
        {
            __Triangle nativeImg = (__Triangle) Marshal.PtrToStructure(from, typeof(__Triangle));
            CopyOut(ref nativeImg, ref to);
        }

        public static void StaticCopyOut(System.IntPtr from, ref common.Triangle to)
        {
            __Triangle nativeImg = (__Triangle) Marshal.PtrToStructure(from, typeof(__Triangle));
            CopyOut(ref nativeImg, ref to);
        }

        public static void CopyOut(ref __Triangle from, ref common.Triangle to)
        {
            if (to == null) {
                to = new common.Triangle();
            }
            common.__Vector3Marshaler.CopyOut(ref from.p1, ref to.p1);
            common.__Vector3Marshaler.CopyOut(ref from.p2, ref to.p2);
            common.__Vector3Marshaler.CopyOut(ref from.p3, ref to.p3);
        }

    }
    #endregion

}

