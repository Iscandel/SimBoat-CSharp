//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChLinkGear : ChLinkLock {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnDerived;

  internal ChLinkGear(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChLinkGear_SWIGSmartPtrUpcast(cPtr), true) {
    swigCMemOwnDerived = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChLinkGear obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwnDerived) {
          swigCMemOwnDerived = false;
          corePINVOKE.delete_ChLinkGear(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public ChLinkGear() : this(corePINVOKE.new_ChLinkGear__SWIG_0(), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChLinkGear(ChLinkGear other) : this(corePINVOKE.new_ChLinkGear__SWIG_1(ChLinkGear.getCPtr(other)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override void UpdateTime(double mytime) {
    corePINVOKE.ChLinkGear_UpdateTime(swigCPtr, mytime);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double Get_tau() {
    double ret = corePINVOKE.ChLinkGear_Get_tau(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void Set_tau(double mset) {
    corePINVOKE.ChLinkGear_Set_tau__SWIG_0(swigCPtr, mset);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Set_tau(double mz1, double mz2) {
    corePINVOKE.ChLinkGear_Set_tau__SWIG_1(swigCPtr, mz1, mz2);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double Get_alpha() {
    double ret = corePINVOKE.ChLinkGear_Get_alpha(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void Set_alpha(double mset) {
    corePINVOKE.ChLinkGear_Set_alpha(swigCPtr, mset);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double Get_beta() {
    double ret = corePINVOKE.ChLinkGear_Get_beta(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void Set_beta(double mset) {
    corePINVOKE.ChLinkGear_Set_beta(swigCPtr, mset);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double Get_phase() {
    double ret = corePINVOKE.ChLinkGear_Get_phase(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void Set_phase(double mset) {
    corePINVOKE.ChLinkGear_Set_phase(swigCPtr, mset);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public bool Get_epicyclic() {
    bool ret = corePINVOKE.ChLinkGear_Get_epicyclic(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void Set_epicyclic(bool mset) {
    corePINVOKE.ChLinkGear_Set_epicyclic(swigCPtr, mset);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Set_checkphase(bool mset) {
    corePINVOKE.ChLinkGear_Set_checkphase(swigCPtr, mset);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public bool Get_checkphase() {
    bool ret = corePINVOKE.ChLinkGear_Get_checkphase(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double Get_a1() {
    double ret = corePINVOKE.ChLinkGear_Get_a1(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double Get_a2() {
    double ret = corePINVOKE.ChLinkGear_Get_a2(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void Reset_a1a2() {
    corePINVOKE.ChLinkGear_Reset_a1a2(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double Get_r1() {
    double ret = corePINVOKE.ChLinkGear_Get_r1(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double Get_r2() {
    double ret = corePINVOKE.ChLinkGear_Get_r2(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChFrameD Get_local_shaft1() {
    ChFrameD ret = new ChFrameD(corePINVOKE.ChLinkGear_Get_local_shaft1(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void Set_local_shaft1(ChFrameD mf) {
    corePINVOKE.ChLinkGear_Set_local_shaft1(swigCPtr, ChFrameD.getCPtr(mf));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChFrameD Get_local_shaft2() {
    ChFrameD ret = new ChFrameD(corePINVOKE.ChLinkGear_Get_local_shaft2(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void Set_local_shaft2(ChFrameD mf) {
    corePINVOKE.ChLinkGear_Set_local_shaft2(swigCPtr, ChFrameD.getCPtr(mf));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChVectorD Get_shaft_dir1() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChLinkGear_Get_shaft_dir1(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD Get_shaft_dir2() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChLinkGear_Get_shaft_dir2(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD Get_shaft_pos1() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChLinkGear_Get_shaft_pos1(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD Get_shaft_pos2() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChLinkGear_Get_shaft_pos2(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public override void ArchiveOUT(SWIGTYPE_p_ChArchiveOut marchive) {
    corePINVOKE.ChLinkGear_ArchiveOUT(swigCPtr, SWIGTYPE_p_ChArchiveOut.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override void ArchiveIN(SWIGTYPE_p_chrono__ChArchiveIn marchive) {
    corePINVOKE.ChLinkGear_ArchiveIN(swigCPtr, SWIGTYPE_p_chrono__ChArchiveIn.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

}