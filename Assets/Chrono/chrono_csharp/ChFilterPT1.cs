//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChFilterPT1 : ChAnalogueFilter {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;

  internal ChFilterPT1(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChFilterPT1_SWIGUpcast(cPtr), cMemoryOwn) {
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChFilterPT1 obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          corePINVOKE.delete_ChFilterPT1(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public ChFilterPT1() : this(corePINVOKE.new_ChFilterPT1__SWIG_0(), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChFilterPT1(double step, double T1, double Kpt1) : this(corePINVOKE.new_ChFilterPT1__SWIG_1(step, T1, Kpt1), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChFilterPT1(double step, double T1) : this(corePINVOKE.new_ChFilterPT1__SWIG_2(step, T1), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChFilterPT1(double step) : this(corePINVOKE.new_ChFilterPT1__SWIG_3(step), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override void Reset() {
    corePINVOKE.ChFilterPT1_Reset(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Config(double step, double T1, double Kpt1) {
    corePINVOKE.ChFilterPT1_Config__SWIG_0(swigCPtr, step, T1, Kpt1);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Config(double step, double T1) {
    corePINVOKE.ChFilterPT1_Config__SWIG_1(swigCPtr, step, T1);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Config(double step) {
    corePINVOKE.ChFilterPT1_Config__SWIG_2(swigCPtr, step);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override double Filter(double u) {
    double ret = corePINVOKE.ChFilterPT1_Filter(swigCPtr, u);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

}
