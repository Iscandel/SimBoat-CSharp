//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChLoadBase : ChObj {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnDerived;

  internal ChLoadBase(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChLoadBase_SWIGSmartPtrUpcast(cPtr), true) {
    swigCMemOwnDerived = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChLoadBase obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwnDerived) {
          swigCMemOwnDerived = false;
          corePINVOKE.delete_ChLoadBase(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public ChLoadBase() : this(corePINVOKE.new_ChLoadBase(), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    SwigDirectorConnect();
  }

  public virtual int LoadGet_ndof_x() {
    int ret = corePINVOKE.ChLoadBase_LoadGet_ndof_x(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public virtual int LoadGet_ndof_w() {
    int ret = corePINVOKE.ChLoadBase_LoadGet_ndof_w(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public virtual void LoadGetStateBlock_x(ChState mD) {
    corePINVOKE.ChLoadBase_LoadGetStateBlock_x(swigCPtr, ChState.getCPtr(mD));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void LoadGetStateBlock_w(ChStateDelta mD) {
    corePINVOKE.ChLoadBase_LoadGetStateBlock_w(swigCPtr, ChStateDelta.getCPtr(mD));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void LoadStateIncrement(ChState x, ChStateDelta dw, ChState x_new) {
    corePINVOKE.ChLoadBase_LoadStateIncrement(swigCPtr, ChState.getCPtr(x), ChStateDelta.getCPtr(dw), ChState.getCPtr(x_new));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual int LoadGet_field_ncoords() {
    int ret = corePINVOKE.ChLoadBase_LoadGet_field_ncoords(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public virtual void ComputeQ(ChState state_x, ChStateDelta state_w) {
    corePINVOKE.ChLoadBase_ComputeQ(swigCPtr, ChState.getCPtr(state_x), ChStateDelta.getCPtr(state_w));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChLoadJacobians GetJacobians() {
    global::System.IntPtr cPtr = corePINVOKE.ChLoadBase_GetJacobians(swigCPtr);
    ChLoadJacobians ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChLoadJacobians(cPtr, false);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public virtual void CreateJacobianMatrices() {
    corePINVOKE.ChLoadBase_CreateJacobianMatrices(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void Update(double time) {
    if (SwigDerivedClassHasMethod("Update", swigMethodTypes10)) corePINVOKE.ChLoadBase_UpdateSwigExplicitChLoadBase(swigCPtr, time); else corePINVOKE.ChLoadBase_Update(swigCPtr, time);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual bool IsStiff() {
    bool ret = corePINVOKE.ChLoadBase_IsStiff(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public virtual void LoadIntLoadResidual_F(ChVectorDynamicD R, double c) {
    corePINVOKE.ChLoadBase_LoadIntLoadResidual_F(swigCPtr, ChVectorDynamicD.getCPtr(R), c);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void InjectKRMmatrices(SWIGTYPE_p_ChSystemDescriptor mdescriptor) {
    if (SwigDerivedClassHasMethod("InjectKRMmatrices", swigMethodTypes13)) corePINVOKE.ChLoadBase_InjectKRMmatricesSwigExplicitChLoadBase(swigCPtr, SWIGTYPE_p_ChSystemDescriptor.getCPtr(mdescriptor)); else corePINVOKE.ChLoadBase_InjectKRMmatrices(swigCPtr, SWIGTYPE_p_ChSystemDescriptor.getCPtr(mdescriptor));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    if (SwigDerivedClassHasMethod("KRMmatricesLoad", swigMethodTypes14)) corePINVOKE.ChLoadBase_KRMmatricesLoadSwigExplicitChLoadBase(swigCPtr, Kfactor, Rfactor, Mfactor); else corePINVOKE.ChLoadBase_KRMmatricesLoad(swigCPtr, Kfactor, Rfactor, Mfactor);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  private void SwigDirectorConnect() {
    if (SwigDerivedClassHasMethod("ArchiveOUT", swigMethodTypes0))
      swigDelegate0 = new SwigDelegateChLoadBase_0(SwigDirectorMethodArchiveOUT);
    if (SwigDerivedClassHasMethod("ArchiveIN", swigMethodTypes1))
      swigDelegate1 = new SwigDelegateChLoadBase_1(SwigDirectorMethodArchiveIN);
    if (SwigDerivedClassHasMethod("LoadGet_ndof_x", swigMethodTypes2))
      swigDelegate2 = new SwigDelegateChLoadBase_2(SwigDirectorMethodLoadGet_ndof_x);
    if (SwigDerivedClassHasMethod("LoadGet_ndof_w", swigMethodTypes3))
      swigDelegate3 = new SwigDelegateChLoadBase_3(SwigDirectorMethodLoadGet_ndof_w);
    if (SwigDerivedClassHasMethod("LoadGetStateBlock_x", swigMethodTypes4))
      swigDelegate4 = new SwigDelegateChLoadBase_4(SwigDirectorMethodLoadGetStateBlock_x);
    if (SwigDerivedClassHasMethod("LoadGetStateBlock_w", swigMethodTypes5))
      swigDelegate5 = new SwigDelegateChLoadBase_5(SwigDirectorMethodLoadGetStateBlock_w);
    if (SwigDerivedClassHasMethod("LoadStateIncrement", swigMethodTypes6))
      swigDelegate6 = new SwigDelegateChLoadBase_6(SwigDirectorMethodLoadStateIncrement);
    if (SwigDerivedClassHasMethod("LoadGet_field_ncoords", swigMethodTypes7))
      swigDelegate7 = new SwigDelegateChLoadBase_7(SwigDirectorMethodLoadGet_field_ncoords);
    if (SwigDerivedClassHasMethod("ComputeQ", swigMethodTypes8))
      swigDelegate8 = new SwigDelegateChLoadBase_8(SwigDirectorMethodComputeQ);
    if (SwigDerivedClassHasMethod("CreateJacobianMatrices", swigMethodTypes9))
      swigDelegate9 = new SwigDelegateChLoadBase_9(SwigDirectorMethodCreateJacobianMatrices);
    if (SwigDerivedClassHasMethod("Update", swigMethodTypes10))
      swigDelegate10 = new SwigDelegateChLoadBase_10(SwigDirectorMethodUpdate);
    if (SwigDerivedClassHasMethod("IsStiff", swigMethodTypes11))
      swigDelegate11 = new SwigDelegateChLoadBase_11(SwigDirectorMethodIsStiff);
    if (SwigDerivedClassHasMethod("LoadIntLoadResidual_F", swigMethodTypes12))
      swigDelegate12 = new SwigDelegateChLoadBase_12(SwigDirectorMethodLoadIntLoadResidual_F);
    if (SwigDerivedClassHasMethod("InjectKRMmatrices", swigMethodTypes13))
      swigDelegate13 = new SwigDelegateChLoadBase_13(SwigDirectorMethodInjectKRMmatrices);
    if (SwigDerivedClassHasMethod("KRMmatricesLoad", swigMethodTypes14))
      swigDelegate14 = new SwigDelegateChLoadBase_14(SwigDirectorMethodKRMmatricesLoad);
    corePINVOKE.ChLoadBase_director_connect(swigCPtr, swigDelegate0, swigDelegate1, swigDelegate2, swigDelegate3, swigDelegate4, swigDelegate5, swigDelegate6, swigDelegate7, swigDelegate8, swigDelegate9, swigDelegate10, swigDelegate11, swigDelegate12, swigDelegate13, swigDelegate14);
  }

  private bool SwigDerivedClassHasMethod(string methodName, global::System.Type[] methodTypes) {
    global::System.Reflection.MethodInfo methodInfo = this.GetType().GetMethod(methodName, global::System.Reflection.BindingFlags.Public | global::System.Reflection.BindingFlags.NonPublic | global::System.Reflection.BindingFlags.Instance, null, methodTypes, null);
    bool hasDerivedMethod = methodInfo.DeclaringType.IsSubclassOf(typeof(ChLoadBase));
    return hasDerivedMethod;
  }

  private void SwigDirectorMethodArchiveOUT(global::System.IntPtr marchive) {
    ArchiveOUT(new SWIGTYPE_p_ChArchiveOut(marchive, false));
  }

  private void SwigDirectorMethodArchiveIN(global::System.IntPtr marchive) {
    ArchiveIN(new SWIGTYPE_p_chrono__ChArchiveIn(marchive, false));
  }

  private int SwigDirectorMethodLoadGet_ndof_x() {
    return LoadGet_ndof_x();
  }

  private int SwigDirectorMethodLoadGet_ndof_w() {
    return LoadGet_ndof_w();
  }

  private void SwigDirectorMethodLoadGetStateBlock_x(global::System.IntPtr mD) {
    LoadGetStateBlock_x(new ChState(mD, false));
  }

  private void SwigDirectorMethodLoadGetStateBlock_w(global::System.IntPtr mD) {
    LoadGetStateBlock_w(new ChStateDelta(mD, false));
  }

  private void SwigDirectorMethodLoadStateIncrement(global::System.IntPtr x, global::System.IntPtr dw, global::System.IntPtr x_new) {
    LoadStateIncrement(new ChState(x, false), new ChStateDelta(dw, false), new ChState(x_new, false));
  }

  private int SwigDirectorMethodLoadGet_field_ncoords() {
    return LoadGet_field_ncoords();
  }

  private void SwigDirectorMethodComputeQ(global::System.IntPtr state_x, global::System.IntPtr state_w) {
    ComputeQ((state_x == global::System.IntPtr.Zero) ? null : new ChState(state_x, false), (state_w == global::System.IntPtr.Zero) ? null : new ChStateDelta(state_w, false));
  }

  private void SwigDirectorMethodCreateJacobianMatrices() {
    CreateJacobianMatrices();
  }

  private void SwigDirectorMethodUpdate(double time) {
    Update(time);
  }

  private bool SwigDirectorMethodIsStiff() {
    return IsStiff();
  }

  private void SwigDirectorMethodLoadIntLoadResidual_F(global::System.IntPtr R, double c) {
    LoadIntLoadResidual_F(new ChVectorDynamicD(R, false), c);
  }

  private void SwigDirectorMethodInjectKRMmatrices(global::System.IntPtr mdescriptor) {
    InjectKRMmatrices(new SWIGTYPE_p_ChSystemDescriptor(mdescriptor, false));
  }

  private void SwigDirectorMethodKRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
  }

  public delegate void SwigDelegateChLoadBase_0(global::System.IntPtr marchive);
  public delegate void SwigDelegateChLoadBase_1(global::System.IntPtr marchive);
  public delegate int SwigDelegateChLoadBase_2();
  public delegate int SwigDelegateChLoadBase_3();
  public delegate void SwigDelegateChLoadBase_4(global::System.IntPtr mD);
  public delegate void SwigDelegateChLoadBase_5(global::System.IntPtr mD);
  public delegate void SwigDelegateChLoadBase_6(global::System.IntPtr x, global::System.IntPtr dw, global::System.IntPtr x_new);
  public delegate int SwigDelegateChLoadBase_7();
  public delegate void SwigDelegateChLoadBase_8(global::System.IntPtr state_x, global::System.IntPtr state_w);
  public delegate void SwigDelegateChLoadBase_9();
  public delegate void SwigDelegateChLoadBase_10(double time);
  public delegate bool SwigDelegateChLoadBase_11();
  public delegate void SwigDelegateChLoadBase_12(global::System.IntPtr R, double c);
  public delegate void SwigDelegateChLoadBase_13(global::System.IntPtr mdescriptor);
  public delegate void SwigDelegateChLoadBase_14(double Kfactor, double Rfactor, double Mfactor);

  private SwigDelegateChLoadBase_0 swigDelegate0;
  private SwigDelegateChLoadBase_1 swigDelegate1;
  private SwigDelegateChLoadBase_2 swigDelegate2;
  private SwigDelegateChLoadBase_3 swigDelegate3;
  private SwigDelegateChLoadBase_4 swigDelegate4;
  private SwigDelegateChLoadBase_5 swigDelegate5;
  private SwigDelegateChLoadBase_6 swigDelegate6;
  private SwigDelegateChLoadBase_7 swigDelegate7;
  private SwigDelegateChLoadBase_8 swigDelegate8;
  private SwigDelegateChLoadBase_9 swigDelegate9;
  private SwigDelegateChLoadBase_10 swigDelegate10;
  private SwigDelegateChLoadBase_11 swigDelegate11;
  private SwigDelegateChLoadBase_12 swigDelegate12;
  private SwigDelegateChLoadBase_13 swigDelegate13;
  private SwigDelegateChLoadBase_14 swigDelegate14;

  private static global::System.Type[] swigMethodTypes0 = new global::System.Type[] { typeof(SWIGTYPE_p_ChArchiveOut) };
  private static global::System.Type[] swigMethodTypes1 = new global::System.Type[] { typeof(SWIGTYPE_p_chrono__ChArchiveIn) };
  private static global::System.Type[] swigMethodTypes2 = new global::System.Type[] {  };
  private static global::System.Type[] swigMethodTypes3 = new global::System.Type[] {  };
  private static global::System.Type[] swigMethodTypes4 = new global::System.Type[] { typeof(ChState) };
  private static global::System.Type[] swigMethodTypes5 = new global::System.Type[] { typeof(ChStateDelta) };
  private static global::System.Type[] swigMethodTypes6 = new global::System.Type[] { typeof(ChState), typeof(ChStateDelta), typeof(ChState) };
  private static global::System.Type[] swigMethodTypes7 = new global::System.Type[] {  };
  private static global::System.Type[] swigMethodTypes8 = new global::System.Type[] { typeof(ChState), typeof(ChStateDelta) };
  private static global::System.Type[] swigMethodTypes9 = new global::System.Type[] {  };
  private static global::System.Type[] swigMethodTypes10 = new global::System.Type[] { typeof(double) };
  private static global::System.Type[] swigMethodTypes11 = new global::System.Type[] {  };
  private static global::System.Type[] swigMethodTypes12 = new global::System.Type[] { typeof(ChVectorDynamicD), typeof(double) };
  private static global::System.Type[] swigMethodTypes13 = new global::System.Type[] { typeof(SWIGTYPE_p_ChSystemDescriptor) };
  private static global::System.Type[] swigMethodTypes14 = new global::System.Type[] { typeof(double), typeof(double), typeof(double) };
}
