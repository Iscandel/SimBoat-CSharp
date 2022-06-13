//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChLinkLimit : global::System.IDisposable {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnBase;

  internal ChLinkLimit(global::System.IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwnBase = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChLinkLimit obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  ~ChLinkLimit() {
    Dispose(false);
  }

  public void Dispose() {
    Dispose(true);
    global::System.GC.SuppressFinalize(this);
  }

  protected virtual void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwnBase) {
          swigCMemOwnBase = false;
          corePINVOKE.delete_ChLinkLimit(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
    }
  }

  public SWIGTYPE_p_ChConstraintTwoBodies constr_upper {
    set {
      corePINVOKE.ChLinkLimit_constr_upper_set(swigCPtr, SWIGTYPE_p_ChConstraintTwoBodies.getCPtr(value));
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      SWIGTYPE_p_ChConstraintTwoBodies ret = new SWIGTYPE_p_ChConstraintTwoBodies(corePINVOKE.ChLinkLimit_constr_upper_get(swigCPtr), true);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public SWIGTYPE_p_ChConstraintTwoBodies constr_lower {
    set {
      corePINVOKE.ChLinkLimit_constr_lower_set(swigCPtr, SWIGTYPE_p_ChConstraintTwoBodies.getCPtr(value));
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      SWIGTYPE_p_ChConstraintTwoBodies ret = new SWIGTYPE_p_ChConstraintTwoBodies(corePINVOKE.ChLinkLimit_constr_lower_get(swigCPtr), true);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public ChLinkLimit() : this(corePINVOKE.new_ChLinkLimit__SWIG_0(), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChLinkLimit(ChLinkLimit other) : this(corePINVOKE.new_ChLinkLimit__SWIG_1(ChLinkLimit.getCPtr(other)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChLinkLimit Clone() {
    global::System.IntPtr cPtr = corePINVOKE.ChLinkLimit_Clone(swigCPtr);
    ChLinkLimit ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChLinkLimit(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public bool IsActive() {
    bool ret = corePINVOKE.ChLinkLimit_IsActive(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetActive(bool val) {
    corePINVOKE.ChLinkLimit_SetActive(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public bool IsPenalty() {
    bool ret = corePINVOKE.ChLinkLimit_IsPenalty(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public bool IsPolar() {
    bool ret = corePINVOKE.ChLinkLimit_IsPolar(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public bool IsRotation() {
    bool ret = corePINVOKE.ChLinkLimit_IsRotation(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetPenalty(bool val) {
    corePINVOKE.ChLinkLimit_SetPenalty(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetPolar(bool val) {
    corePINVOKE.ChLinkLimit_SetPolar(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetRotation(bool val) {
    corePINVOKE.ChLinkLimit_SetRotation(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double GetMax() {
    double ret = corePINVOKE.ChLinkLimit_GetMax(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetMin() {
    double ret = corePINVOKE.ChLinkLimit_GetMin(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetMaxCushion() {
    double ret = corePINVOKE.ChLinkLimit_GetMaxCushion(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetMinCushion() {
    double ret = corePINVOKE.ChLinkLimit_GetMinCushion(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetKmax() {
    double ret = corePINVOKE.ChLinkLimit_GetKmax(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetKmin() {
    double ret = corePINVOKE.ChLinkLimit_GetKmin(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetRmax() {
    double ret = corePINVOKE.ChLinkLimit_GetRmax(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetRmin() {
    double ret = corePINVOKE.ChLinkLimit_GetRmin(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetMaxElastic() {
    double ret = corePINVOKE.ChLinkLimit_GetMaxElastic(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetMinElastic() {
    double ret = corePINVOKE.ChLinkLimit_GetMinElastic(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetMaxPolarAngle(double pol_ang) {
    double ret = corePINVOKE.ChLinkLimit_GetMaxPolarAngle(swigCPtr, pol_ang);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetMax(double val) {
    corePINVOKE.ChLinkLimit_SetMax(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetMin(double val) {
    corePINVOKE.ChLinkLimit_SetMin(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetMaxCushion(double val) {
    corePINVOKE.ChLinkLimit_SetMaxCushion(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetMinCushion(double val) {
    corePINVOKE.ChLinkLimit_SetMinCushion(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetKmax(double val) {
    corePINVOKE.ChLinkLimit_SetKmax(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetKmin(double val) {
    corePINVOKE.ChLinkLimit_SetKmin(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetRmax(double val) {
    corePINVOKE.ChLinkLimit_SetRmax(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetRmin(double val) {
    corePINVOKE.ChLinkLimit_SetRmin(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetMaxElastic(double val) {
    corePINVOKE.ChLinkLimit_SetMaxElastic(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetMinElastic(double val) {
    corePINVOKE.ChLinkLimit_SetMinElastic(swigCPtr, val);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetModulationKmax(ChFunction funct) {
    corePINVOKE.ChLinkLimit_SetModulationKmax(swigCPtr, ChFunction.getCPtr(funct));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetModulationKmin(ChFunction funct) {
    corePINVOKE.ChLinkLimit_SetModulationKmin(swigCPtr, ChFunction.getCPtr(funct));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetModulationRmax(ChFunction funct) {
    corePINVOKE.ChLinkLimit_SetModulationRmax(swigCPtr, ChFunction.getCPtr(funct));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetModulationRmin(ChFunction funct) {
    corePINVOKE.ChLinkLimit_SetModulationRmin(swigCPtr, ChFunction.getCPtr(funct));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetPolarMax(ChFunction funct) {
    corePINVOKE.ChLinkLimit_SetPolarMax(swigCPtr, ChFunction.getCPtr(funct));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChFunction GetModulationKmax() {
    global::System.IntPtr cPtr = corePINVOKE.ChLinkLimit_GetModulationKmax(swigCPtr);
    ChFunction ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChFunction(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChFunction GetModulationKmin() {
    global::System.IntPtr cPtr = corePINVOKE.ChLinkLimit_GetModulationKmin(swigCPtr);
    ChFunction ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChFunction(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChFunction GetModulationRmax() {
    global::System.IntPtr cPtr = corePINVOKE.ChLinkLimit_GetModulationRmax(swigCPtr);
    ChFunction ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChFunction(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChFunction GetModulationRmin() {
    global::System.IntPtr cPtr = corePINVOKE.ChLinkLimit_GetModulationRmin(swigCPtr);
    ChFunction ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChFunction(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChFunction GetPolarMax() {
    global::System.IntPtr cPtr = corePINVOKE.ChLinkLimit_GetPolarMax(swigCPtr);
    ChFunction ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChFunction(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetViolation(double x) {
    double ret = corePINVOKE.ChLinkLimit_GetViolation(swigCPtr, x);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetForce(double x, double x_dt) {
    double ret = corePINVOKE.ChLinkLimit_GetForce(swigCPtr, x, x_dt);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetPolarForce(double x, double x_dt, double pol_ang) {
    double ret = corePINVOKE.ChLinkLimit_GetPolarForce(swigCPtr, x, x_dt, pol_ang);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void ArchiveOUT(SWIGTYPE_p_ChArchiveOut marchive) {
    corePINVOKE.ChLinkLimit_ArchiveOUT(swigCPtr, SWIGTYPE_p_ChArchiveOut.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void ArchiveIN(SWIGTYPE_p_chrono__ChArchiveIn marchive) {
    corePINVOKE.ChLinkLimit_ArchiveIN(swigCPtr, SWIGTYPE_p_chrono__ChArchiveIn.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

}
