//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChParticleRemoverBox : ChParticleProcessor {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;

  internal ChParticleRemoverBox(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChParticleRemoverBox_SWIGUpcast(cPtr), cMemoryOwn) {
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChParticleRemoverBox obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          corePINVOKE.delete_ChParticleRemoverBox(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public ChParticleRemoverBox() : this(corePINVOKE.new_ChParticleRemoverBox(), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBox GetBox() {
    ChBox ret = new ChBox(corePINVOKE.ChParticleRemoverBox_GetBox(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetRemoveOutside(bool minvert) {
    corePINVOKE.ChParticleRemoverBox_SetRemoveOutside(swigCPtr, minvert);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

}
