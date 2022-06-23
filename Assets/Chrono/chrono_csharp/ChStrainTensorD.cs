//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChStrainTensorD : ChVoightTensorD {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;

  internal ChStrainTensorD(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChStrainTensorD_SWIGUpcast(cPtr), cMemoryOwn) {
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChStrainTensorD obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          corePINVOKE.delete_ChStrainTensorD(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public ChStrainTensorD() : this(corePINVOKE.new_ChStrainTensorD(), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void ComputePrincipalStrains(SWIGTYPE_p_double e1, SWIGTYPE_p_double e2, SWIGTYPE_p_double e3) {
    corePINVOKE.ChStrainTensorD_ComputePrincipalStrains(swigCPtr, SWIGTYPE_p_double.getCPtr(e1), SWIGTYPE_p_double.getCPtr(e2), SWIGTYPE_p_double.getCPtr(e3));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void ComputePrincipalStrainsDirections(SWIGTYPE_p_double e1, SWIGTYPE_p_double e2, SWIGTYPE_p_double e3, ChVectorD dir1, ChVectorD dir2, ChVectorD dir3) {
    corePINVOKE.ChStrainTensorD_ComputePrincipalStrainsDirections(swigCPtr, SWIGTYPE_p_double.getCPtr(e1), SWIGTYPE_p_double.getCPtr(e2), SWIGTYPE_p_double.getCPtr(e3), ChVectorD.getCPtr(dir1), ChVectorD.getCPtr(dir2), ChVectorD.getCPtr(dir3));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

}