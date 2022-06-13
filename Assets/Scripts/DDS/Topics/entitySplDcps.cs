using DDS;
using DDS.OpenSplice.CustomMarshalers;
using DDS.OpenSplice.Database;
using DDS.OpenSplice.Kernel;
using System;
using System.Runtime.InteropServices;

namespace entity
{
    #region __EntityInfo
    [StructLayout(LayoutKind.Sequential)]
    public struct __EntityInfo
    {
        public short id;
        public IntPtr type;
    }
    #endregion

    #region __EntityInfoMarshaler
    public sealed class __EntityInfoMarshaler : DDS.OpenSplice.CustomMarshalers.FooDatabaseMarshaler<entity.EntityInfo>
    {
        public static readonly string fullyScopedName = "entity::EntityInfo";

        public override void InitEmbeddedMarshalers(IDomainParticipant participant)
        {
        }

        public override V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, System.IntPtr from, System.IntPtr to)
        {
            GCHandle tmpGCHandle = GCHandle.FromIntPtr(from);
            entity.EntityInfo fromData = tmpGCHandle.Target as entity.EntityInfo;
            return CopyIn(typePtr, fromData, to);
        }

        public V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, entity.EntityInfo from, System.IntPtr to)
        {
            __EntityInfo nativeImg = new __EntityInfo();
            V_COPYIN_RESULT result = CopyIn(typePtr, from, ref nativeImg);
            if (result == V_COPYIN_RESULT.OK)
            {
                Marshal.StructureToPtr(nativeImg, to, false);
            }
            return result;
        }

        public V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, entity.EntityInfo from, ref __EntityInfo to)
        {
            if (from == null) return V_COPYIN_RESULT.INVALID;
            to.id = from.id;
            if (from.type == null) return V_COPYIN_RESULT.INVALID;
            // Unbounded string: bounds check not required...
            if (!Write(c.getBase(typePtr), ref to.type, from.type)) return V_COPYIN_RESULT.OUT_OF_MEMORY;
            return V_COPYIN_RESULT.OK;
        }

        public override void CopyOut(System.IntPtr from, System.IntPtr to)
        {
            __EntityInfo nativeImg = (__EntityInfo) Marshal.PtrToStructure(from, typeof(__EntityInfo));
            GCHandle tmpGCHandleTo = GCHandle.FromIntPtr(to);
            entity.EntityInfo toObj = tmpGCHandleTo.Target as entity.EntityInfo;
            CopyOut(ref nativeImg, ref toObj);
            tmpGCHandleTo.Target = toObj;
        }

        public override void CopyOut(System.IntPtr from, ref entity.EntityInfo to)
        {
            __EntityInfo nativeImg = (__EntityInfo) Marshal.PtrToStructure(from, typeof(__EntityInfo));
            CopyOut(ref nativeImg, ref to);
        }

        public static void StaticCopyOut(System.IntPtr from, ref entity.EntityInfo to)
        {
            __EntityInfo nativeImg = (__EntityInfo) Marshal.PtrToStructure(from, typeof(__EntityInfo));
            CopyOut(ref nativeImg, ref to);
        }

        public static void CopyOut(ref __EntityInfo from, ref entity.EntityInfo to)
        {
            if (to == null) {
                to = new entity.EntityInfo();
            }
            to.id = from.id;
            to.type = ReadString(from.type);
        }

    }
    #endregion

    #region __EntityKinematicsInfo
    [StructLayout(LayoutKind.Sequential)]
    public struct __EntityKinematicsInfo
    {
        public short id;
        public float x;
        public float y;
        public float z;
        public double roll;
        public double pitch;
        public double yaw;
    }
    #endregion

    #region __EntityKinematicsInfoMarshaler
    public sealed class __EntityKinematicsInfoMarshaler : DDS.OpenSplice.CustomMarshalers.FooDatabaseMarshaler<entity.EntityKinematicsInfo>
    {
        public static readonly string fullyScopedName = "entity::EntityKinematicsInfo";

        public override void InitEmbeddedMarshalers(IDomainParticipant participant)
        {
        }

        public override V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, System.IntPtr from, System.IntPtr to)
        {
            GCHandle tmpGCHandle = GCHandle.FromIntPtr(from);
            entity.EntityKinematicsInfo fromData = tmpGCHandle.Target as entity.EntityKinematicsInfo;
            return CopyIn(typePtr, fromData, to);
        }

        public V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, entity.EntityKinematicsInfo from, System.IntPtr to)
        {
            __EntityKinematicsInfo nativeImg = new __EntityKinematicsInfo();
            V_COPYIN_RESULT result = CopyIn(typePtr, from, ref nativeImg);
            if (result == V_COPYIN_RESULT.OK)
            {
                Marshal.StructureToPtr(nativeImg, to, false);
            }
            return result;
        }

        public V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, entity.EntityKinematicsInfo from, ref __EntityKinematicsInfo to)
        {
            if (from == null) return V_COPYIN_RESULT.INVALID;
            to.id = from.id;
            to.x = from.x;
            to.y = from.y;
            to.z = from.z;
            to.roll = from.roll;
            to.pitch = from.pitch;
            to.yaw = from.yaw;
            return V_COPYIN_RESULT.OK;
        }

        public override void CopyOut(System.IntPtr from, System.IntPtr to)
        {
            __EntityKinematicsInfo nativeImg = (__EntityKinematicsInfo) Marshal.PtrToStructure(from, typeof(__EntityKinematicsInfo));
            GCHandle tmpGCHandleTo = GCHandle.FromIntPtr(to);
            entity.EntityKinematicsInfo toObj = tmpGCHandleTo.Target as entity.EntityKinematicsInfo;
            CopyOut(ref nativeImg, ref toObj);
            tmpGCHandleTo.Target = toObj;
        }

        public override void CopyOut(System.IntPtr from, ref entity.EntityKinematicsInfo to)
        {
            __EntityKinematicsInfo nativeImg = (__EntityKinematicsInfo) Marshal.PtrToStructure(from, typeof(__EntityKinematicsInfo));
            CopyOut(ref nativeImg, ref to);
        }

        public static void StaticCopyOut(System.IntPtr from, ref entity.EntityKinematicsInfo to)
        {
            __EntityKinematicsInfo nativeImg = (__EntityKinematicsInfo) Marshal.PtrToStructure(from, typeof(__EntityKinematicsInfo));
            CopyOut(ref nativeImg, ref to);
        }

        public static void CopyOut(ref __EntityKinematicsInfo from, ref entity.EntityKinematicsInfo to)
        {
            if (to == null) {
                to = new entity.EntityKinematicsInfo();
            }
            to.id = from.id;
            to.x = from.x;
            to.y = from.y;
            to.z = from.z;
            to.roll = from.roll;
            to.pitch = from.pitch;
            to.yaw = from.yaw;
        }

    }
    #endregion

    #region __EntityShape
    [StructLayout(LayoutKind.Sequential)]
    public struct __EntityShape
    {
        public short id;
        public IntPtr triangles_list;
    }
    #endregion

    #region __EntityShapeMarshaler
    public sealed class __EntityShapeMarshaler : DDS.OpenSplice.CustomMarshalers.FooDatabaseMarshaler<entity.EntityShape>
    {
        public static readonly string fullyScopedName = "entity::EntityShape";
        private IntPtr attr1Col0Type = IntPtr.Zero;
        private static readonly int attr1Col0Size = 1 * Marshal.SizeOf(typeof(common.__Triangle));
        private common.__TriangleMarshaler attr1Marshaler;

        public override void InitEmbeddedMarshalers(IDomainParticipant participant)
        {
            if (attr1Marshaler == null) {
                attr1Marshaler = DatabaseMarshaler.GetMarshaler(participant, typeof(common.__Triangle)) as common.__TriangleMarshaler;
                if (attr1Marshaler == null) {
                    attr1Marshaler = new common.__TriangleMarshaler();
                    DatabaseMarshaler.Add(participant, typeof(common.__Triangle), attr1Marshaler);
                    attr1Marshaler.InitEmbeddedMarshalers(participant);
                }
            }
        }

        public override V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, System.IntPtr from, System.IntPtr to)
        {
            GCHandle tmpGCHandle = GCHandle.FromIntPtr(from);
            entity.EntityShape fromData = tmpGCHandle.Target as entity.EntityShape;
            return CopyIn(typePtr, fromData, to);
        }

        public V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, entity.EntityShape from, System.IntPtr to)
        {
            __EntityShape nativeImg = new __EntityShape();
            V_COPYIN_RESULT result = CopyIn(typePtr, from, ref nativeImg);
            if (result == V_COPYIN_RESULT.OK)
            {
                Marshal.StructureToPtr(nativeImg, to, false);
            }
            return result;
        }

        public V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, entity.EntityShape from, ref __EntityShape to)
        {
            if (from == null) return V_COPYIN_RESULT.INVALID;
            to.id = from.id;
            if (from.triangles_list == null) return V_COPYIN_RESULT.INVALID;
            int attr1Seq0Length = from.triangles_list.Length;
            // Unbounded sequence: bounds check not required...
            if (attr1Col0Type == IntPtr.Zero) {
                IntPtr memberOwnerType = DDS.OpenSplice.Database.c.resolve(c.getBase(typePtr), fullyScopedName);
                IntPtr specifier = DDS.OpenSplice.Database.c.metaResolveSpecifier(memberOwnerType, "triangles_list");
                IntPtr specifierType = DDS.OpenSplice.Database.c.specifierType(specifier);
                attr1Col0Type = DDS.OpenSplice.Database.c.typeActualType(specifierType);
            }
            IntPtr attr1Seq0Buf = DDS.OpenSplice.Database.c.newSequence(attr1Col0Type, attr1Seq0Length);
            if (attr1Seq0Buf == IntPtr.Zero) return V_COPYIN_RESULT.OUT_OF_MEMORY;
            to.triangles_list = attr1Seq0Buf;
            for (int i0 = 0; i0 < attr1Seq0Length; i0++) {
                V_COPYIN_RESULT result = attr1Marshaler.CopyIn(typePtr, from.triangles_list[i0], attr1Seq0Buf);
                if (result != V_COPYIN_RESULT.OK) return result;
                attr1Seq0Buf = new IntPtr(attr1Seq0Buf.ToInt64() + attr1Col0Size);
            }
            return V_COPYIN_RESULT.OK;
        }

        public override void CopyOut(System.IntPtr from, System.IntPtr to)
        {
            __EntityShape nativeImg = (__EntityShape) Marshal.PtrToStructure(from, typeof(__EntityShape));
            GCHandle tmpGCHandleTo = GCHandle.FromIntPtr(to);
            entity.EntityShape toObj = tmpGCHandleTo.Target as entity.EntityShape;
            CopyOut(ref nativeImg, ref toObj);
            tmpGCHandleTo.Target = toObj;
        }

        public override void CopyOut(System.IntPtr from, ref entity.EntityShape to)
        {
            __EntityShape nativeImg = (__EntityShape) Marshal.PtrToStructure(from, typeof(__EntityShape));
            CopyOut(ref nativeImg, ref to);
        }

        public static void StaticCopyOut(System.IntPtr from, ref entity.EntityShape to)
        {
            __EntityShape nativeImg = (__EntityShape) Marshal.PtrToStructure(from, typeof(__EntityShape));
            CopyOut(ref nativeImg, ref to);
        }

        public static void CopyOut(ref __EntityShape from, ref entity.EntityShape to)
        {
            if (to == null) {
                to = new entity.EntityShape();
            }
            to.id = from.id;
            IntPtr attr1Seq0Buf = from.triangles_list;
            int attr1Seq0Length = DDS.OpenSplice.Database.c.arraySize(attr1Seq0Buf);
            if (to.triangles_list == null || to.triangles_list.Length != attr1Seq0Length) {
                common.Triangle[] target = new common.Triangle[attr1Seq0Length];
                initObjectSeq(to.triangles_list, target);
                to.triangles_list = target;
            }
            for (int i0 = 0; i0 < attr1Seq0Length; i0++) {
                common.__TriangleMarshaler.StaticCopyOut(attr1Seq0Buf, ref to.triangles_list[i0]);
                attr1Seq0Buf = new IntPtr(attr1Seq0Buf.ToInt64() + attr1Col0Size);
            }
        }

    }
    #endregion

}

