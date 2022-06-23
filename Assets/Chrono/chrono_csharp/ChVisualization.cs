//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChVisualization : ChAsset {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnDerived;

  internal ChVisualization(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChVisualization_SWIGSmartPtrUpcast(cPtr), true) {
    swigCMemOwnDerived = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChVisualization obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwnDerived) {
          swigCMemOwnDerived = false;
          corePINVOKE.delete_ChVisualization(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public void SetVisible(bool mv) {
    corePINVOKE.ChVisualization_SetVisible(swigCPtr, mv);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public bool IsVisible() {
    bool ret = corePINVOKE.ChVisualization_IsVisible(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetColor(ChColor mc) {
    corePINVOKE.ChVisualization_SetColor(swigCPtr, ChColor.getCPtr(mc));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChColor GetColor() {
    ChColor ret = new ChColor(corePINVOKE.ChVisualization_GetColor(swigCPtr), false);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetFading(float mc) {
    corePINVOKE.ChVisualization_SetFading(swigCPtr, mc);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public float GetFading() {
    float ret = corePINVOKE.ChVisualization_GetFading(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetStatic(bool val) {
    corePINVOKE.ChVisualization_SetStatic(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public bool IsStatic() {
    bool ret = corePINVOKE.ChVisualization_IsStatic(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD Pos {
    set {
      corePINVOKE.ChVisualization_Pos_set(swigCPtr, ChVectorD.getCPtr(value));
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      global::System.IntPtr cPtr = corePINVOKE.ChVisualization_Pos_get(swigCPtr);
      ChVectorD ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChVectorD(cPtr, false);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public ChMatrix33D Rot {
    set {
      corePINVOKE.ChVisualization_Rot_set(swigCPtr, ChMatrix33D.getCPtr(value));
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      global::System.IntPtr cPtr = corePINVOKE.ChVisualization_Rot_get(swigCPtr);
      ChMatrix33D ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChMatrix33D(cPtr, false);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public material_list material_list {
    set {
      corePINVOKE.ChVisualization_material_list_set(swigCPtr, material_list.getCPtr(value));
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      global::System.IntPtr cPtr = corePINVOKE.ChVisualization_material_list_get(swigCPtr);
      material_list ret = (cPtr == global::System.IntPtr.Zero) ? null : new material_list(cPtr, false);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

}