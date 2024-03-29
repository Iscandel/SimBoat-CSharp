//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChMatrix33D : global::System.IDisposable {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  protected bool swigCMemOwn;

  internal ChMatrix33D(global::System.IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChMatrix33D obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  ~ChMatrix33D() {
    Dispose(false);
  }

  public void Dispose() {
    Dispose(true);
    global::System.GC.SuppressFinalize(this);
  }

  protected virtual void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          corePINVOKE.delete_ChMatrix33D(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
    }
  }

  public ChMatrix33D() : this(corePINVOKE.new_ChMatrix33D__SWIG_0(), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChMatrix33D(double val) : this(corePINVOKE.new_ChMatrix33D__SWIG_1(val), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChMatrix33D(ChVectorD v) : this(corePINVOKE.new_ChMatrix33D__SWIG_2(ChVectorD.getCPtr(v)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChMatrix33D(ChVectorD diag, ChVectorD off_diag) : this(corePINVOKE.new_ChMatrix33D__SWIG_3(ChVectorD.getCPtr(diag), ChVectorD.getCPtr(off_diag)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChMatrix33D(ChQuaternionD q) : this(corePINVOKE.new_ChMatrix33D__SWIG_4(ChQuaternionD.getCPtr(q)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChMatrix33D(double angle, ChVectorD axis) : this(corePINVOKE.new_ChMatrix33D__SWIG_5(angle, ChVectorD.getCPtr(axis)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChMatrix33D(ChVectorD X, ChVectorD Y, ChVectorD Z) : this(corePINVOKE.new_ChMatrix33D__SWIG_6(ChVectorD.getCPtr(X), ChVectorD.getCPtr(Y), ChVectorD.getCPtr(Z)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Set_A_quaternion(ChQuaternionD quat) {
    corePINVOKE.ChMatrix33D_Set_A_quaternion(swigCPtr, ChQuaternionD.getCPtr(quat));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Set_A_Eulero(ChVectorD angles) {
    corePINVOKE.ChMatrix33D_Set_A_Eulero(swigCPtr, ChVectorD.getCPtr(angles));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Set_A_Cardano(ChVectorD angles) {
    corePINVOKE.ChMatrix33D_Set_A_Cardano(swigCPtr, ChVectorD.getCPtr(angles));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Set_A_Hpb(ChVectorD angles) {
    corePINVOKE.ChMatrix33D_Set_A_Hpb(swigCPtr, ChVectorD.getCPtr(angles));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Set_A_Rxyz(ChVectorD xyz) {
    corePINVOKE.ChMatrix33D_Set_A_Rxyz(swigCPtr, ChVectorD.getCPtr(xyz));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Set_A_Rodriguez(ChVectorD rod) {
    corePINVOKE.ChMatrix33D_Set_A_Rodriguez(swigCPtr, ChVectorD.getCPtr(rod));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Set_A_axis(ChVectorD X, ChVectorD Y, ChVectorD Z) {
    corePINVOKE.ChMatrix33D_Set_A_axis(swigCPtr, ChVectorD.getCPtr(X), ChVectorD.getCPtr(Y), ChVectorD.getCPtr(Z));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Set_A_Xdir(ChVectorD Xdir, ChVectorD Vsingular) {
    corePINVOKE.ChMatrix33D_Set_A_Xdir__SWIG_0(swigCPtr, ChVectorD.getCPtr(Xdir), ChVectorD.getCPtr(Vsingular));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Set_A_Xdir(ChVectorD Xdir) {
    corePINVOKE.ChMatrix33D_Set_A_Xdir__SWIG_1(swigCPtr, ChVectorD.getCPtr(Xdir));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChVectorD Get_A_Xaxis() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChMatrix33D_Get_A_Xaxis(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD Get_A_Yaxis() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChMatrix33D_Get_A_Yaxis(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD Get_A_Zaxis() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChMatrix33D_Get_A_Zaxis(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChQuaternionD Get_A_quaternion() {
    ChQuaternionD ret = new ChQuaternionD(corePINVOKE.ChMatrix33D_Get_A_quaternion(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD Get_A_Eulero() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChMatrix33D_Get_A_Eulero(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD Get_A_Cardano() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChMatrix33D_Get_A_Cardano(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD Get_A_Hpb() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChMatrix33D_Get_A_Hpb(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD Get_A_Rxyz() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChMatrix33D_Get_A_Rxyz(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD Get_A_Rodriguez() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChMatrix33D_Get_A_Rodriguez(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD GetAx() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChMatrix33D_GetAx(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SelfAdjointEigenSolve(ChMatrix33D evec, SWIGTYPE_p_chrono__ChVectorNT_double_3_t evals) {
    corePINVOKE.ChMatrix33D_SelfAdjointEigenSolve(swigCPtr, ChMatrix33D.getCPtr(evec), SWIGTYPE_p_chrono__ChVectorNT_double_3_t.getCPtr(evals));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double getitem(int i, int j) {
    double ret = corePINVOKE.ChMatrix33D_getitem(swigCPtr, i, j);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setitem(int i, int j, double v) {
    corePINVOKE.ChMatrix33D_setitem(swigCPtr, i, j, v);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public int GetRows() {
    int ret = corePINVOKE.ChMatrix33D_GetRows(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public int GetColumns() {
    int ret = corePINVOKE.ChMatrix33D_GetColumns(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

}
