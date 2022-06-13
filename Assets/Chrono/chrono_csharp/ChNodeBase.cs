//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChNodeBase : global::System.IDisposable {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnBase;

  internal ChNodeBase(global::System.IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwnBase = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChNodeBase obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  ~ChNodeBase() {
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
          corePINVOKE.delete_ChNodeBase(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
    }
  }

  public virtual int Get_ndof_x() {
    int ret = corePINVOKE.ChNodeBase_Get_ndof_x(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public virtual int Get_ndof_w() {
    int ret = corePINVOKE.ChNodeBase_Get_ndof_w(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public uint NodeGetOffset_x() {
    uint ret = corePINVOKE.ChNodeBase_NodeGetOffset_x(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public uint NodeGetOffset_w() {
    uint ret = corePINVOKE.ChNodeBase_NodeGetOffset_w(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void NodeSetOffset_x(uint moff) {
    corePINVOKE.ChNodeBase_NodeSetOffset_x(swigCPtr, moff);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void NodeSetOffset_w(uint moff) {
    corePINVOKE.ChNodeBase_NodeSetOffset_w(swigCPtr, moff);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void NodeIntStateGather(uint off_x, ChState x, uint off_v, ChStateDelta v, SWIGTYPE_p_double T) {
    corePINVOKE.ChNodeBase_NodeIntStateGather(swigCPtr, off_x, ChState.getCPtr(x), off_v, ChStateDelta.getCPtr(v), SWIGTYPE_p_double.getCPtr(T));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void NodeIntStateScatter(uint off_x, ChState x, uint off_v, ChStateDelta v, double T) {
    corePINVOKE.ChNodeBase_NodeIntStateScatter(swigCPtr, off_x, ChState.getCPtr(x), off_v, ChStateDelta.getCPtr(v), T);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void NodeIntStateGatherAcceleration(uint off_a, ChStateDelta a) {
    corePINVOKE.ChNodeBase_NodeIntStateGatherAcceleration(swigCPtr, off_a, ChStateDelta.getCPtr(a));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void NodeIntStateScatterAcceleration(uint off_a, ChStateDelta a) {
    corePINVOKE.ChNodeBase_NodeIntStateScatterAcceleration(swigCPtr, off_a, ChStateDelta.getCPtr(a));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void NodeIntStateIncrement(uint off_x, ChState x_new, ChState x, uint off_v, ChStateDelta Dv) {
    corePINVOKE.ChNodeBase_NodeIntStateIncrement(swigCPtr, off_x, ChState.getCPtr(x_new), ChState.getCPtr(x), off_v, ChStateDelta.getCPtr(Dv));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void NodeIntLoadResidual_F(uint off, ChVectorDynamicD R, double c) {
    corePINVOKE.ChNodeBase_NodeIntLoadResidual_F(swigCPtr, off, ChVectorDynamicD.getCPtr(R), c);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void NodeIntLoadResidual_Mv(uint off, ChVectorDynamicD R, ChVectorDynamicD w, double c) {
    corePINVOKE.ChNodeBase_NodeIntLoadResidual_Mv(swigCPtr, off, ChVectorDynamicD.getCPtr(R), ChVectorDynamicD.getCPtr(w), c);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void NodeIntToDescriptor(uint off_v, ChStateDelta v, ChVectorDynamicD R) {
    corePINVOKE.ChNodeBase_NodeIntToDescriptor(swigCPtr, off_v, ChStateDelta.getCPtr(v), ChVectorDynamicD.getCPtr(R));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void NodeIntFromDescriptor(uint off_v, ChStateDelta v) {
    corePINVOKE.ChNodeBase_NodeIntFromDescriptor(swigCPtr, off_v, ChStateDelta.getCPtr(v));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void InjectVariables(SWIGTYPE_p_ChSystemDescriptor mdescriptor) {
    corePINVOKE.ChNodeBase_InjectVariables(swigCPtr, SWIGTYPE_p_ChSystemDescriptor.getCPtr(mdescriptor));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void VariablesFbReset() {
    corePINVOKE.ChNodeBase_VariablesFbReset(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void VariablesFbLoadForces(double factor) {
    corePINVOKE.ChNodeBase_VariablesFbLoadForces__SWIG_0(swigCPtr, factor);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void VariablesFbLoadForces() {
    corePINVOKE.ChNodeBase_VariablesFbLoadForces__SWIG_1(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void VariablesQbLoadSpeed() {
    corePINVOKE.ChNodeBase_VariablesQbLoadSpeed(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void VariablesFbIncrementMq() {
    corePINVOKE.ChNodeBase_VariablesFbIncrementMq(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void VariablesQbSetSpeed(double step) {
    corePINVOKE.ChNodeBase_VariablesQbSetSpeed__SWIG_0(swigCPtr, step);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void VariablesQbSetSpeed() {
    corePINVOKE.ChNodeBase_VariablesQbSetSpeed__SWIG_1(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void VariablesQbIncrementPosition(double step) {
    corePINVOKE.ChNodeBase_VariablesQbIncrementPosition(swigCPtr, step);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void ArchiveOUT(SWIGTYPE_p_ChArchiveOut marchive) {
    corePINVOKE.ChNodeBase_ArchiveOUT(swigCPtr, SWIGTYPE_p_ChArchiveOut.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void ArchiveIN(SWIGTYPE_p_chrono__ChArchiveIn marchive) {
    corePINVOKE.ChNodeBase_ArchiveIN(swigCPtr, SWIGTYPE_p_chrono__ChArchiveIn.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

}
