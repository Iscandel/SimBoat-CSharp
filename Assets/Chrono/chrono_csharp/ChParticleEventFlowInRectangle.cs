//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChParticleEventFlowInRectangle : ChParticleEventTrigger {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnDerived;

  internal ChParticleEventFlowInRectangle(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChParticleEventFlowInRectangle_SWIGSmartPtrUpcast(cPtr), true) {
    swigCMemOwnDerived = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChParticleEventFlowInRectangle obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwnDerived) {
          swigCMemOwnDerived = false;
          corePINVOKE.delete_ChParticleEventFlowInRectangle(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public ChParticleEventFlowInRectangle(double mXsize, double mYsize) : this(corePINVOKE.new_ChParticleEventFlowInRectangle__SWIG_0(mXsize, mYsize), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChParticleEventFlowInRectangle(double mXsize) : this(corePINVOKE.new_ChParticleEventFlowInRectangle__SWIG_1(mXsize), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChParticleEventFlowInRectangle() : this(corePINVOKE.new_ChParticleEventFlowInRectangle__SWIG_2(), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override bool TriggerEvent(ChBody mbody, ChSystem msystem) {
    bool ret = corePINVOKE.ChParticleEventFlowInRectangle_TriggerEvent(swigCPtr, ChBody.getCPtr(mbody), ChSystem.getCPtr(msystem));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public override void SetupPostProcess(ChSystem msystem) {
    corePINVOKE.ChParticleEventFlowInRectangle_SetupPostProcess(swigCPtr, ChSystem.getCPtr(msystem));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double Xsize {
    set {
      corePINVOKE.ChParticleEventFlowInRectangle_Xsize_set(swigCPtr, value);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      double ret = corePINVOKE.ChParticleEventFlowInRectangle_Xsize_get(swigCPtr);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public double Ysize {
    set {
      corePINVOKE.ChParticleEventFlowInRectangle_Ysize_set(swigCPtr, value);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      double ret = corePINVOKE.ChParticleEventFlowInRectangle_Ysize_get(swigCPtr);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public double margin {
    set {
      corePINVOKE.ChParticleEventFlowInRectangle_margin_set(swigCPtr, value);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      double ret = corePINVOKE.ChParticleEventFlowInRectangle_margin_get(swigCPtr);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public ChCoordsysD rectangle_csys {
    set {
      corePINVOKE.ChParticleEventFlowInRectangle_rectangle_csys_set(swigCPtr, ChCoordsysD.getCPtr(value));
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      global::System.IntPtr cPtr = corePINVOKE.ChParticleEventFlowInRectangle_rectangle_csys_get(swigCPtr);
      ChCoordsysD ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChCoordsysD(cPtr, false);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public ChVectorD last_intersectionUV {
    set {
      corePINVOKE.ChParticleEventFlowInRectangle_last_intersectionUV_set(swigCPtr, ChVectorD.getCPtr(value));
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      global::System.IntPtr cPtr = corePINVOKE.ChParticleEventFlowInRectangle_last_intersectionUV_get(swigCPtr);
      ChVectorD ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChVectorD(cPtr, false);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

}
