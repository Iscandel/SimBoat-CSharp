//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChTimestepperEulerImplicitLinearized : ChTimestepperIIorder {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnDerived;

  internal ChTimestepperEulerImplicitLinearized(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChTimestepperEulerImplicitLinearized_SWIGSmartPtrUpcast(cPtr), true) {
    swigCMemOwnDerived = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChTimestepperEulerImplicitLinearized obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwnDerived) {
          swigCMemOwnDerived = false;
          corePINVOKE.delete_ChTimestepperEulerImplicitLinearized(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public ChTimestepperEulerImplicitLinearized(ChIntegrableIIorder intgr) : this(corePINVOKE.new_ChTimestepperEulerImplicitLinearized__SWIG_0(ChIntegrableIIorder.getCPtr(intgr)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChTimestepperEulerImplicitLinearized() : this(corePINVOKE.new_ChTimestepperEulerImplicitLinearized__SWIG_1(), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override ChTimestepper.Type GetType() {
    ChTimestepper.Type ret = (ChTimestepper.Type)corePINVOKE.ChTimestepperEulerImplicitLinearized_GetType(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public override void Advance(double dt) {
    corePINVOKE.ChTimestepperEulerImplicitLinearized_Advance(swigCPtr, dt);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

}
