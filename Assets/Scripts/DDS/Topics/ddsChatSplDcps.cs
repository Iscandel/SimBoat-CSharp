using DDS;
using DDS.OpenSplice.CustomMarshalers;
using DDS.OpenSplice.Database;
using DDS.OpenSplice.Kernel;
using System;
using System.Runtime.InteropServices;

namespace chat
{
    #region __ChatMsg
    [StructLayout(LayoutKind.Sequential)]
    public struct __ChatMsg
    {
        public short id;
        public int cpt;
        public IntPtr text;
    }
    #endregion

    #region __ChatMsgMarshaler
    public sealed class __ChatMsgMarshaler : DDS.OpenSplice.CustomMarshalers.FooDatabaseMarshaler<chat.ChatMsg>
    {
        public static readonly string fullyScopedName = "chat::ChatMsg";

        public override void InitEmbeddedMarshalers(IDomainParticipant participant)
        {
        }

        public override V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, System.IntPtr from, System.IntPtr to)
        {
            GCHandle tmpGCHandle = GCHandle.FromIntPtr(from);
            chat.ChatMsg fromData = tmpGCHandle.Target as chat.ChatMsg;
            return CopyIn(typePtr, fromData, to);
        }

        public V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, chat.ChatMsg from, System.IntPtr to)
        {
            __ChatMsg nativeImg = new __ChatMsg();
            V_COPYIN_RESULT result = CopyIn(typePtr, from, ref nativeImg);
            if (result == V_COPYIN_RESULT.OK)
            {
                Marshal.StructureToPtr(nativeImg, to, false);
            }
            return result;
        }

        public V_COPYIN_RESULT CopyIn(System.IntPtr typePtr, chat.ChatMsg from, ref __ChatMsg to)
        {
            if (from == null) return V_COPYIN_RESULT.INVALID;
            to.id = from.id;
            to.cpt = from.cpt;
            if (from.text == null) return V_COPYIN_RESULT.INVALID;
            // Unbounded string: bounds check not required...
            if (!Write(c.getBase(typePtr), ref to.text, from.text)) return V_COPYIN_RESULT.OUT_OF_MEMORY;
            return V_COPYIN_RESULT.OK;
        }

        public override void CopyOut(System.IntPtr from, System.IntPtr to)
        {
            __ChatMsg nativeImg = (__ChatMsg) Marshal.PtrToStructure(from, typeof(__ChatMsg));
            GCHandle tmpGCHandleTo = GCHandle.FromIntPtr(to);
            chat.ChatMsg toObj = tmpGCHandleTo.Target as chat.ChatMsg;
            CopyOut(ref nativeImg, ref toObj);
            tmpGCHandleTo.Target = toObj;
        }

        public override void CopyOut(System.IntPtr from, ref chat.ChatMsg to)
        {
            __ChatMsg nativeImg = (__ChatMsg) Marshal.PtrToStructure(from, typeof(__ChatMsg));
            CopyOut(ref nativeImg, ref to);
        }

        public static void StaticCopyOut(System.IntPtr from, ref chat.ChatMsg to)
        {
            __ChatMsg nativeImg = (__ChatMsg) Marshal.PtrToStructure(from, typeof(__ChatMsg));
            CopyOut(ref nativeImg, ref to);
        }

        public static void CopyOut(ref __ChatMsg from, ref chat.ChatMsg to)
        {
            if (to == null) {
                to = new chat.ChatMsg();
            }
            to.id = from.id;
            to.cpt = from.cpt;
            to.text = ReadString(from.text);
        }

    }
    #endregion

}

