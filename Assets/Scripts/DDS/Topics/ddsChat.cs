using DDS;
using System.Runtime.InteropServices;

namespace chat
{
    #region ChatMsg
    [StructLayout(LayoutKind.Sequential)]
    public sealed class ChatMsg
    {
        public short id;
        public int cpt;
        public string text = string.Empty;
    };
    #endregion

}

