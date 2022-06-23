//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChShaftsTorsionSpringDamper : ChShaftsLoad {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnDerived;

  internal ChShaftsTorsionSpringDamper(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChShaftsTorsionSpringDamper_SWIGSmartPtrUpcast(cPtr), true) {
    swigCMemOwnDerived = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChShaftsTorsionSpringDamper obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwnDerived) {
          swigCMemOwnDerived = false;
          corePINVOKE.delete_ChShaftsTorsionSpringDamper(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public ChShaftsTorsionSpringDamper(ChShaft mbodyA, ChShaft mbodyB, double mstiffness, double mdamping) : this(corePINVOKE.new_ChShaftsTorsionSpringDamper(ChShaft.getCPtr(mbodyA), ChShaft.getCPtr(mbodyB), mstiffness, mdamping), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetTorsionalStiffness(double mstiffness) {
    corePINVOKE.ChShaftsTorsionSpringDamper_SetTorsionalStiffness(swigCPtr, mstiffness);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double GetTorsionalStiffness() {
    double ret = corePINVOKE.ChShaftsTorsionSpringDamper_GetTorsionalStiffness(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetTorsionalDamping(double mdamping) {
    corePINVOKE.ChShaftsTorsionSpringDamper_SetTorsionalDamping(swigCPtr, mdamping);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double GetTorsionalDamping() {
    double ret = corePINVOKE.ChShaftsTorsionSpringDamper_GetTorsionalDamping(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetRestPhase(double mphase) {
    corePINVOKE.ChShaftsTorsionSpringDamper_SetRestPhase(swigCPtr, mphase);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double GetRestPhase() {
    double ret = corePINVOKE.ChShaftsTorsionSpringDamper_GetRestPhase(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

}