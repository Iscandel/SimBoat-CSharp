//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChFunction_Setpoint : ChFunction {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnDerived;

  internal ChFunction_Setpoint(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChFunction_Setpoint_SWIGSmartPtrUpcast(cPtr), true) {
    swigCMemOwnDerived = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChFunction_Setpoint obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwnDerived) {
          swigCMemOwnDerived = false;
          corePINVOKE.delete_ChFunction_Setpoint(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public ChFunction_Setpoint() : this(corePINVOKE.new_ChFunction_Setpoint__SWIG_0(), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    SwigDirectorConnect();
  }

  public ChFunction_Setpoint(ChFunction_Setpoint other) : this(corePINVOKE.new_ChFunction_Setpoint__SWIG_1(ChFunction_Setpoint.getCPtr(other)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    SwigDirectorConnect();
  }

  public override double Get_y(double x) {
    double ret = (SwigDerivedClassHasMethod("Get_y", swigMethodTypes1) ? corePINVOKE.ChFunction_Setpoint_Get_ySwigExplicitChFunction_Setpoint(swigCPtr, x) : corePINVOKE.ChFunction_Setpoint_Get_y(swigCPtr, x));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public override double Get_y_dx(double x) {
    double ret = (SwigDerivedClassHasMethod("Get_y_dx", swigMethodTypes2) ? corePINVOKE.ChFunction_Setpoint_Get_y_dxSwigExplicitChFunction_Setpoint(swigCPtr, x) : corePINVOKE.ChFunction_Setpoint_Get_y_dx(swigCPtr, x));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public override double Get_y_dxdx(double x) {
    double ret = (SwigDerivedClassHasMethod("Get_y_dxdx", swigMethodTypes3) ? corePINVOKE.ChFunction_Setpoint_Get_y_dxdxSwigExplicitChFunction_Setpoint(swigCPtr, x) : corePINVOKE.ChFunction_Setpoint_Get_y_dxdx(swigCPtr, x));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public virtual void SetSetpoint(double setpoint, double x) {
    if (SwigDerivedClassHasMethod("SetSetpoint", swigMethodTypes26)) corePINVOKE.ChFunction_Setpoint_SetSetpointSwigExplicitChFunction_Setpoint(swigCPtr, setpoint, x); else corePINVOKE.ChFunction_Setpoint_SetSetpoint(swigCPtr, setpoint, x);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void SetSetpointAndDerivatives(double setpoint, double setpoint_dx, double setpoint_dxdx) {
    if (SwigDerivedClassHasMethod("SetSetpointAndDerivatives", swigMethodTypes27)) corePINVOKE.ChFunction_Setpoint_SetSetpointAndDerivativesSwigExplicitChFunction_Setpoint(swigCPtr, setpoint, setpoint_dx, setpoint_dxdx); else corePINVOKE.ChFunction_Setpoint_SetSetpointAndDerivatives(swigCPtr, setpoint, setpoint_dx, setpoint_dxdx);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double GetSetpoint() {
    double ret = corePINVOKE.ChFunction_Setpoint_GetSetpoint(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public override void Update(double x) {
    if (SwigDerivedClassHasMethod("Update", swigMethodTypes8)) corePINVOKE.ChFunction_Setpoint_UpdateSwigExplicitChFunction_Setpoint(swigCPtr, x); else corePINVOKE.ChFunction_Setpoint_Update(swigCPtr, x);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override void ArchiveOUT(SWIGTYPE_p_ChArchiveOut marchive) {
    if (SwigDerivedClassHasMethod("ArchiveOUT", swigMethodTypes19)) corePINVOKE.ChFunction_Setpoint_ArchiveOUTSwigExplicitChFunction_Setpoint(swigCPtr, SWIGTYPE_p_ChArchiveOut.getCPtr(marchive)); else corePINVOKE.ChFunction_Setpoint_ArchiveOUT(swigCPtr, SWIGTYPE_p_ChArchiveOut.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override void ArchiveIN(SWIGTYPE_p_chrono__ChArchiveIn marchive) {
    if (SwigDerivedClassHasMethod("ArchiveIN", swigMethodTypes20)) corePINVOKE.ChFunction_Setpoint_ArchiveINSwigExplicitChFunction_Setpoint(swigCPtr, SWIGTYPE_p_chrono__ChArchiveIn.getCPtr(marchive)); else corePINVOKE.ChFunction_Setpoint_ArchiveIN(swigCPtr, SWIGTYPE_p_chrono__ChArchiveIn.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  private void SwigDirectorConnect() {
    if (SwigDerivedClassHasMethod("Get_Type", swigMethodTypes0))
      swigDelegate0 = new SwigDelegateChFunction_Setpoint_0(SwigDirectorMethodGet_Type);
    if (SwigDerivedClassHasMethod("Get_y", swigMethodTypes1))
      swigDelegate1 = new SwigDelegateChFunction_Setpoint_1(SwigDirectorMethodGet_y);
    if (SwigDerivedClassHasMethod("Get_y_dx", swigMethodTypes2))
      swigDelegate2 = new SwigDelegateChFunction_Setpoint_2(SwigDirectorMethodGet_y_dx);
    if (SwigDerivedClassHasMethod("Get_y_dxdx", swigMethodTypes3))
      swigDelegate3 = new SwigDelegateChFunction_Setpoint_3(SwigDirectorMethodGet_y_dxdx);
    if (SwigDerivedClassHasMethod("Get_weight", swigMethodTypes4))
      swigDelegate4 = new SwigDelegateChFunction_Setpoint_4(SwigDirectorMethodGet_weight);
    if (SwigDerivedClassHasMethod("Estimate_x_range", swigMethodTypes5))
      swigDelegate5 = new SwigDelegateChFunction_Setpoint_5(SwigDirectorMethodEstimate_x_range);
    if (SwigDerivedClassHasMethod("Estimate_y_range", swigMethodTypes6))
      swigDelegate6 = new SwigDelegateChFunction_Setpoint_6(SwigDirectorMethodEstimate_y_range);
    if (SwigDerivedClassHasMethod("Get_y_dN", swigMethodTypes7))
      swigDelegate7 = new SwigDelegateChFunction_Setpoint_7(SwigDirectorMethodGet_y_dN);
    if (SwigDerivedClassHasMethod("Update", swigMethodTypes8))
      swigDelegate8 = new SwigDelegateChFunction_Setpoint_8(SwigDirectorMethodUpdate);
    if (SwigDerivedClassHasMethod("Compute_max", swigMethodTypes9))
      swigDelegate9 = new SwigDelegateChFunction_Setpoint_9(SwigDirectorMethodCompute_max);
    if (SwigDerivedClassHasMethod("Compute_min", swigMethodTypes10))
      swigDelegate10 = new SwigDelegateChFunction_Setpoint_10(SwigDirectorMethodCompute_min);
    if (SwigDerivedClassHasMethod("Compute_mean", swigMethodTypes11))
      swigDelegate11 = new SwigDelegateChFunction_Setpoint_11(SwigDirectorMethodCompute_mean);
    if (SwigDerivedClassHasMethod("Compute_sqrmean", swigMethodTypes12))
      swigDelegate12 = new SwigDelegateChFunction_Setpoint_12(SwigDirectorMethodCompute_sqrmean);
    if (SwigDerivedClassHasMethod("Compute_int", swigMethodTypes13))
      swigDelegate13 = new SwigDelegateChFunction_Setpoint_13(SwigDirectorMethodCompute_int);
    if (SwigDerivedClassHasMethod("Get_Ca_pos", swigMethodTypes14))
      swigDelegate14 = new SwigDelegateChFunction_Setpoint_14(SwigDirectorMethodGet_Ca_pos);
    if (SwigDerivedClassHasMethod("Get_Ca_neg", swigMethodTypes15))
      swigDelegate15 = new SwigDelegateChFunction_Setpoint_15(SwigDirectorMethodGet_Ca_neg);
    if (SwigDerivedClassHasMethod("Get_Cv", swigMethodTypes16))
      swigDelegate16 = new SwigDelegateChFunction_Setpoint_16(SwigDirectorMethodGet_Cv);
    if (SwigDerivedClassHasMethod("HandleNumber", swigMethodTypes17))
      swigDelegate17 = new SwigDelegateChFunction_Setpoint_17(SwigDirectorMethodHandleNumber);
    if (SwigDerivedClassHasMethod("HandleAccess", swigMethodTypes18))
      swigDelegate18 = new SwigDelegateChFunction_Setpoint_18(SwigDirectorMethodHandleAccess);
    if (SwigDerivedClassHasMethod("ArchiveOUT", swigMethodTypes19))
      swigDelegate19 = new SwigDelegateChFunction_Setpoint_19(SwigDirectorMethodArchiveOUT);
    if (SwigDerivedClassHasMethod("ArchiveIN", swigMethodTypes20))
      swigDelegate20 = new SwigDelegateChFunction_Setpoint_20(SwigDirectorMethodArchiveIN);
    if (SwigDerivedClassHasMethod("FilePostscriptPlot", swigMethodTypes21))
      swigDelegate21 = new SwigDelegateChFunction_Setpoint_21(SwigDirectorMethodFilePostscriptPlot);
    if (SwigDerivedClassHasMethod("FileAsciiPairsSave", swigMethodTypes22))
      swigDelegate22 = new SwigDelegateChFunction_Setpoint_22(SwigDirectorMethodFileAsciiPairsSave__SWIG_0);
    if (SwigDerivedClassHasMethod("FileAsciiPairsSave", swigMethodTypes23))
      swigDelegate23 = new SwigDelegateChFunction_Setpoint_23(SwigDirectorMethodFileAsciiPairsSave__SWIG_1);
    if (SwigDerivedClassHasMethod("FileAsciiPairsSave", swigMethodTypes24))
      swigDelegate24 = new SwigDelegateChFunction_Setpoint_24(SwigDirectorMethodFileAsciiPairsSave__SWIG_2);
    if (SwigDerivedClassHasMethod("FileAsciiPairsSave", swigMethodTypes25))
      swigDelegate25 = new SwigDelegateChFunction_Setpoint_25(SwigDirectorMethodFileAsciiPairsSave__SWIG_3);
    if (SwigDerivedClassHasMethod("SetSetpoint", swigMethodTypes26))
      swigDelegate26 = new SwigDelegateChFunction_Setpoint_26(SwigDirectorMethodSetSetpoint);
    if (SwigDerivedClassHasMethod("SetSetpointAndDerivatives", swigMethodTypes27))
      swigDelegate27 = new SwigDelegateChFunction_Setpoint_27(SwigDirectorMethodSetSetpointAndDerivatives);
    corePINVOKE.ChFunction_Setpoint_director_connect(swigCPtr, swigDelegate0, swigDelegate1, swigDelegate2, swigDelegate3, swigDelegate4, swigDelegate5, swigDelegate6, swigDelegate7, swigDelegate8, swigDelegate9, swigDelegate10, swigDelegate11, swigDelegate12, swigDelegate13, swigDelegate14, swigDelegate15, swigDelegate16, swigDelegate17, swigDelegate18, swigDelegate19, swigDelegate20, swigDelegate21, swigDelegate22, swigDelegate23, swigDelegate24, swigDelegate25, swigDelegate26, swigDelegate27);
  }

  private bool SwigDerivedClassHasMethod(string methodName, global::System.Type[] methodTypes) {
    global::System.Reflection.MethodInfo methodInfo = this.GetType().GetMethod(methodName, global::System.Reflection.BindingFlags.Public | global::System.Reflection.BindingFlags.NonPublic | global::System.Reflection.BindingFlags.Instance, null, methodTypes, null);
    bool hasDerivedMethod = methodInfo.DeclaringType.IsSubclassOf(typeof(ChFunction_Setpoint));
    return hasDerivedMethod;
  }

  private int SwigDirectorMethodGet_Type() {
    return (int)Get_Type();
  }

  private double SwigDirectorMethodGet_y(double x) {
    return Get_y(x);
  }

  private double SwigDirectorMethodGet_y_dx(double x) {
    return Get_y_dx(x);
  }

  private double SwigDirectorMethodGet_y_dxdx(double x) {
    return Get_y_dxdx(x);
  }

  private double SwigDirectorMethodGet_weight(double x) {
    return Get_weight(x);
  }

  private void SwigDirectorMethodEstimate_x_range(global::System.IntPtr xmin, global::System.IntPtr xmax) {
    Estimate_x_range(new SWIGTYPE_p_double(xmin, false), new SWIGTYPE_p_double(xmax, false));
  }

  private void SwigDirectorMethodEstimate_y_range(double xmin, double xmax, global::System.IntPtr ymin, global::System.IntPtr ymax, int derivate) {
    Estimate_y_range(xmin, xmax, new SWIGTYPE_p_double(ymin, false), new SWIGTYPE_p_double(ymax, false), derivate);
  }

  private double SwigDirectorMethodGet_y_dN(double x, int derivate) {
    return Get_y_dN(x, derivate);
  }

  private void SwigDirectorMethodUpdate(double x) {
    Update(x);
  }

  private double SwigDirectorMethodCompute_max(double xmin, double xmax, double sampling_step, int derivate) {
    return Compute_max(xmin, xmax, sampling_step, derivate);
  }

  private double SwigDirectorMethodCompute_min(double xmin, double xmax, double sampling_step, int derivate) {
    return Compute_min(xmin, xmax, sampling_step, derivate);
  }

  private double SwigDirectorMethodCompute_mean(double xmin, double xmax, double sampling_step, int derivate) {
    return Compute_mean(xmin, xmax, sampling_step, derivate);
  }

  private double SwigDirectorMethodCompute_sqrmean(double xmin, double xmax, double sampling_step, int derivate) {
    return Compute_sqrmean(xmin, xmax, sampling_step, derivate);
  }

  private double SwigDirectorMethodCompute_int(double xmin, double xmax, double sampling_step, int derivate) {
    return Compute_int(xmin, xmax, sampling_step, derivate);
  }

  private double SwigDirectorMethodGet_Ca_pos() {
    return Get_Ca_pos();
  }

  private double SwigDirectorMethodGet_Ca_neg() {
    return Get_Ca_neg();
  }

  private double SwigDirectorMethodGet_Cv() {
    return Get_Cv();
  }

  private int SwigDirectorMethodHandleNumber() {
    return HandleNumber();
  }

  private bool SwigDirectorMethodHandleAccess(int handle_id, double mx, double my, bool set_mode) {
    return HandleAccess(handle_id, mx, my, set_mode);
  }

  private void SwigDirectorMethodArchiveOUT(global::System.IntPtr marchive) {
    ArchiveOUT(new SWIGTYPE_p_ChArchiveOut(marchive, false));
  }

  private void SwigDirectorMethodArchiveIN(global::System.IntPtr marchive) {
    ArchiveIN(new SWIGTYPE_p_chrono__ChArchiveIn(marchive, false));
  }

  private int SwigDirectorMethodFilePostscriptPlot(global::System.IntPtr m_file, int plotY, int plotDY, int plotDDY) {
    return FilePostscriptPlot((m_file == global::System.IntPtr.Zero) ? null : new SWIGTYPE_p_ChFile_ps(m_file, false), plotY, plotDY, plotDDY);
  }

  private int SwigDirectorMethodFileAsciiPairsSave__SWIG_0(global::System.IntPtr m_file, double xmin, double xmax, int msamples) {
    return FileAsciiPairsSave(new ChStreamOutAscii(m_file, false), xmin, xmax, msamples);
  }

  private int SwigDirectorMethodFileAsciiPairsSave__SWIG_1(global::System.IntPtr m_file, double xmin, double xmax) {
    return FileAsciiPairsSave(new ChStreamOutAscii(m_file, false), xmin, xmax);
  }

  private int SwigDirectorMethodFileAsciiPairsSave__SWIG_2(global::System.IntPtr m_file, double xmin) {
    return FileAsciiPairsSave(new ChStreamOutAscii(m_file, false), xmin);
  }

  private int SwigDirectorMethodFileAsciiPairsSave__SWIG_3(global::System.IntPtr m_file) {
    return FileAsciiPairsSave(new ChStreamOutAscii(m_file, false));
  }

  private void SwigDirectorMethodSetSetpoint(double setpoint, double x) {
    SetSetpoint(setpoint, x);
  }

  private void SwigDirectorMethodSetSetpointAndDerivatives(double setpoint, double setpoint_dx, double setpoint_dxdx) {
    SetSetpointAndDerivatives(setpoint, setpoint_dx, setpoint_dxdx);
  }

  public delegate int SwigDelegateChFunction_Setpoint_0();
  public delegate double SwigDelegateChFunction_Setpoint_1(double x);
  public delegate double SwigDelegateChFunction_Setpoint_2(double x);
  public delegate double SwigDelegateChFunction_Setpoint_3(double x);
  public delegate double SwigDelegateChFunction_Setpoint_4(double x);
  public delegate void SwigDelegateChFunction_Setpoint_5(global::System.IntPtr xmin, global::System.IntPtr xmax);
  public delegate void SwigDelegateChFunction_Setpoint_6(double xmin, double xmax, global::System.IntPtr ymin, global::System.IntPtr ymax, int derivate);
  public delegate double SwigDelegateChFunction_Setpoint_7(double x, int derivate);
  public delegate void SwigDelegateChFunction_Setpoint_8(double x);
  public delegate double SwigDelegateChFunction_Setpoint_9(double xmin, double xmax, double sampling_step, int derivate);
  public delegate double SwigDelegateChFunction_Setpoint_10(double xmin, double xmax, double sampling_step, int derivate);
  public delegate double SwigDelegateChFunction_Setpoint_11(double xmin, double xmax, double sampling_step, int derivate);
  public delegate double SwigDelegateChFunction_Setpoint_12(double xmin, double xmax, double sampling_step, int derivate);
  public delegate double SwigDelegateChFunction_Setpoint_13(double xmin, double xmax, double sampling_step, int derivate);
  public delegate double SwigDelegateChFunction_Setpoint_14();
  public delegate double SwigDelegateChFunction_Setpoint_15();
  public delegate double SwigDelegateChFunction_Setpoint_16();
  public delegate int SwigDelegateChFunction_Setpoint_17();
  public delegate bool SwigDelegateChFunction_Setpoint_18(int handle_id, double mx, double my, bool set_mode);
  public delegate void SwigDelegateChFunction_Setpoint_19(global::System.IntPtr marchive);
  public delegate void SwigDelegateChFunction_Setpoint_20(global::System.IntPtr marchive);
  public delegate int SwigDelegateChFunction_Setpoint_21(global::System.IntPtr m_file, int plotY, int plotDY, int plotDDY);
  public delegate int SwigDelegateChFunction_Setpoint_22(global::System.IntPtr m_file, double xmin, double xmax, int msamples);
  public delegate int SwigDelegateChFunction_Setpoint_23(global::System.IntPtr m_file, double xmin, double xmax);
  public delegate int SwigDelegateChFunction_Setpoint_24(global::System.IntPtr m_file, double xmin);
  public delegate int SwigDelegateChFunction_Setpoint_25(global::System.IntPtr m_file);
  public delegate void SwigDelegateChFunction_Setpoint_26(double setpoint, double x);
  public delegate void SwigDelegateChFunction_Setpoint_27(double setpoint, double setpoint_dx, double setpoint_dxdx);

  private SwigDelegateChFunction_Setpoint_0 swigDelegate0;
  private SwigDelegateChFunction_Setpoint_1 swigDelegate1;
  private SwigDelegateChFunction_Setpoint_2 swigDelegate2;
  private SwigDelegateChFunction_Setpoint_3 swigDelegate3;
  private SwigDelegateChFunction_Setpoint_4 swigDelegate4;
  private SwigDelegateChFunction_Setpoint_5 swigDelegate5;
  private SwigDelegateChFunction_Setpoint_6 swigDelegate6;
  private SwigDelegateChFunction_Setpoint_7 swigDelegate7;
  private SwigDelegateChFunction_Setpoint_8 swigDelegate8;
  private SwigDelegateChFunction_Setpoint_9 swigDelegate9;
  private SwigDelegateChFunction_Setpoint_10 swigDelegate10;
  private SwigDelegateChFunction_Setpoint_11 swigDelegate11;
  private SwigDelegateChFunction_Setpoint_12 swigDelegate12;
  private SwigDelegateChFunction_Setpoint_13 swigDelegate13;
  private SwigDelegateChFunction_Setpoint_14 swigDelegate14;
  private SwigDelegateChFunction_Setpoint_15 swigDelegate15;
  private SwigDelegateChFunction_Setpoint_16 swigDelegate16;
  private SwigDelegateChFunction_Setpoint_17 swigDelegate17;
  private SwigDelegateChFunction_Setpoint_18 swigDelegate18;
  private SwigDelegateChFunction_Setpoint_19 swigDelegate19;
  private SwigDelegateChFunction_Setpoint_20 swigDelegate20;
  private SwigDelegateChFunction_Setpoint_21 swigDelegate21;
  private SwigDelegateChFunction_Setpoint_22 swigDelegate22;
  private SwigDelegateChFunction_Setpoint_23 swigDelegate23;
  private SwigDelegateChFunction_Setpoint_24 swigDelegate24;
  private SwigDelegateChFunction_Setpoint_25 swigDelegate25;
  private SwigDelegateChFunction_Setpoint_26 swigDelegate26;
  private SwigDelegateChFunction_Setpoint_27 swigDelegate27;

  private static global::System.Type[] swigMethodTypes0 = new global::System.Type[] {  };
  private static global::System.Type[] swigMethodTypes1 = new global::System.Type[] { typeof(double) };
  private static global::System.Type[] swigMethodTypes2 = new global::System.Type[] { typeof(double) };
  private static global::System.Type[] swigMethodTypes3 = new global::System.Type[] { typeof(double) };
  private static global::System.Type[] swigMethodTypes4 = new global::System.Type[] { typeof(double) };
  private static global::System.Type[] swigMethodTypes5 = new global::System.Type[] { typeof(SWIGTYPE_p_double), typeof(SWIGTYPE_p_double) };
  private static global::System.Type[] swigMethodTypes6 = new global::System.Type[] { typeof(double), typeof(double), typeof(SWIGTYPE_p_double), typeof(SWIGTYPE_p_double), typeof(int) };
  private static global::System.Type[] swigMethodTypes7 = new global::System.Type[] { typeof(double), typeof(int) };
  private static global::System.Type[] swigMethodTypes8 = new global::System.Type[] { typeof(double) };
  private static global::System.Type[] swigMethodTypes9 = new global::System.Type[] { typeof(double), typeof(double), typeof(double), typeof(int) };
  private static global::System.Type[] swigMethodTypes10 = new global::System.Type[] { typeof(double), typeof(double), typeof(double), typeof(int) };
  private static global::System.Type[] swigMethodTypes11 = new global::System.Type[] { typeof(double), typeof(double), typeof(double), typeof(int) };
  private static global::System.Type[] swigMethodTypes12 = new global::System.Type[] { typeof(double), typeof(double), typeof(double), typeof(int) };
  private static global::System.Type[] swigMethodTypes13 = new global::System.Type[] { typeof(double), typeof(double), typeof(double), typeof(int) };
  private static global::System.Type[] swigMethodTypes14 = new global::System.Type[] {  };
  private static global::System.Type[] swigMethodTypes15 = new global::System.Type[] {  };
  private static global::System.Type[] swigMethodTypes16 = new global::System.Type[] {  };
  private static global::System.Type[] swigMethodTypes17 = new global::System.Type[] {  };
  private static global::System.Type[] swigMethodTypes18 = new global::System.Type[] { typeof(int), typeof(double), typeof(double), typeof(bool) };
  private static global::System.Type[] swigMethodTypes19 = new global::System.Type[] { typeof(SWIGTYPE_p_ChArchiveOut) };
  private static global::System.Type[] swigMethodTypes20 = new global::System.Type[] { typeof(SWIGTYPE_p_chrono__ChArchiveIn) };
  private static global::System.Type[] swigMethodTypes21 = new global::System.Type[] { typeof(SWIGTYPE_p_ChFile_ps), typeof(int), typeof(int), typeof(int) };
  private static global::System.Type[] swigMethodTypes22 = new global::System.Type[] { typeof(ChStreamOutAscii), typeof(double), typeof(double), typeof(int) };
  private static global::System.Type[] swigMethodTypes23 = new global::System.Type[] { typeof(ChStreamOutAscii), typeof(double), typeof(double) };
  private static global::System.Type[] swigMethodTypes24 = new global::System.Type[] { typeof(ChStreamOutAscii), typeof(double) };
  private static global::System.Type[] swigMethodTypes25 = new global::System.Type[] { typeof(ChStreamOutAscii) };
  private static global::System.Type[] swigMethodTypes26 = new global::System.Type[] { typeof(double), typeof(double) };
  private static global::System.Type[] swigMethodTypes27 = new global::System.Type[] { typeof(double), typeof(double), typeof(double) };
}
