//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChForce : ChObj {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnDerived;

  internal ChForce(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChForce_SWIGSmartPtrUpcast(cPtr), true) {
    swigCMemOwnDerived = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChForce obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwnDerived) {
          swigCMemOwnDerived = false;
          corePINVOKE.delete_ChForce(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public ChForce() : this(corePINVOKE.new_ChForce__SWIG_0(), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChForce(ChForce other) : this(corePINVOKE.new_ChForce__SWIG_1(ChForce.getCPtr(other)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBody GetBody() {
    global::System.IntPtr cPtr = corePINVOKE.ChForce_GetBody(swigCPtr);
    ChBody ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChBody(cPtr, false);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetBody(ChBody newRB) {
    corePINVOKE.ChForce_SetBody(swigCPtr, ChBody.getCPtr(newRB));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetMode(ChForce.ForceType m_mode) {
    corePINVOKE.ChForce_SetMode(swigCPtr, (int)m_mode);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChForce.ForceType GetMode() {
    ChForce.ForceType ret = (ChForce.ForceType)corePINVOKE.ChForce_GetMode(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetAlign(ChForce.AlignmentFrame m_align) {
    corePINVOKE.ChForce_SetAlign(swigCPtr, (int)m_align);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChForce.AlignmentFrame GetAlign() {
    ChForce.AlignmentFrame ret = (ChForce.AlignmentFrame)corePINVOKE.ChForce_GetAlign(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetFrame(ChForce.ReferenceFrame m_frame) {
    corePINVOKE.ChForce_SetFrame(swigCPtr, (int)m_frame);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChForce.ReferenceFrame GetFrame() {
    ChForce.ReferenceFrame ret = (ChForce.ReferenceFrame)corePINVOKE.ChForce_GetFrame(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD GetVpoint() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChForce_GetVpoint(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD GetVrelpoint() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChForce_GetVrelpoint(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetVpoint(ChVectorD mypoint) {
    corePINVOKE.ChForce_SetVpoint(swigCPtr, ChVectorD.getCPtr(mypoint));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetVrelpoint(ChVectorD myrelpoint) {
    corePINVOKE.ChForce_SetVrelpoint(swigCPtr, ChVectorD.getCPtr(myrelpoint));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChVectorD GetDir() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChForce_GetDir(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD GetRelDir() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChForce_GetRelDir(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetDir(ChVectorD newf) {
    corePINVOKE.ChForce_SetDir(swigCPtr, ChVectorD.getCPtr(newf));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetRelDir(ChVectorD newf) {
    corePINVOKE.ChForce_SetRelDir(swigCPtr, ChVectorD.getCPtr(newf));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetMforce(double newf) {
    corePINVOKE.ChForce_SetMforce(swigCPtr, newf);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double GetMforce() {
    double ret = corePINVOKE.ChForce_GetMforce(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetModulation(ChFunction m_funct) {
    corePINVOKE.ChForce_SetModulation(swigCPtr, ChFunction.getCPtr(m_funct));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChFunction GetModulation() {
    global::System.IntPtr cPtr = corePINVOKE.ChForce_GetModulation(swigCPtr);
    ChFunction ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChFunction(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetMove_x(ChFunction m_funct) {
    corePINVOKE.ChForce_SetMove_x(swigCPtr, ChFunction.getCPtr(m_funct));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChFunction GetMove_x() {
    global::System.IntPtr cPtr = corePINVOKE.ChForce_GetMove_x(swigCPtr);
    ChFunction ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChFunction(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetMove_y(ChFunction m_funct) {
    corePINVOKE.ChForce_SetMove_y(swigCPtr, ChFunction.getCPtr(m_funct));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChFunction GetMove_y() {
    global::System.IntPtr cPtr = corePINVOKE.ChForce_GetMove_y(swigCPtr);
    ChFunction ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChFunction(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetMove_z(ChFunction m_funct) {
    corePINVOKE.ChForce_SetMove_z(swigCPtr, ChFunction.getCPtr(m_funct));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChFunction GetMove_z() {
    global::System.IntPtr cPtr = corePINVOKE.ChForce_GetMove_z(swigCPtr);
    ChFunction ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChFunction(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetF_x(ChFunction m_funct) {
    corePINVOKE.ChForce_SetF_x(swigCPtr, ChFunction.getCPtr(m_funct));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChFunction GetF_x() {
    global::System.IntPtr cPtr = corePINVOKE.ChForce_GetF_x(swigCPtr);
    ChFunction ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChFunction(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetF_y(ChFunction m_funct) {
    corePINVOKE.ChForce_SetF_y(swigCPtr, ChFunction.getCPtr(m_funct));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChFunction GetF_y() {
    global::System.IntPtr cPtr = corePINVOKE.ChForce_GetF_y(swigCPtr);
    ChFunction ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChFunction(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void SetF_z(ChFunction m_funct) {
    corePINVOKE.ChForce_SetF_z(swigCPtr, ChFunction.getCPtr(m_funct));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChFunction GetF_z() {
    global::System.IntPtr cPtr = corePINVOKE.ChForce_GetF_z(swigCPtr);
    ChFunction ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChFunction(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD GetForce() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChForce_GetForce(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorD GetRelForce() {
    ChVectorD ret = new ChVectorD(corePINVOKE.ChForce_GetRelForce(swigCPtr), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double GetForceMod() {
    double ret = corePINVOKE.ChForce_GetForceMod(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public SWIGTYPE_p_chrono__ChVectorNT_double_7_t GetQf() {
    SWIGTYPE_p_chrono__ChVectorNT_double_7_t ret = new SWIGTYPE_p_chrono__ChVectorNT_double_7_t(corePINVOKE.ChForce_GetQf(swigCPtr), false);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void GetBodyForceTorque(ChVectorD body_force, ChVectorD body_torque) {
    corePINVOKE.ChForce_GetBodyForceTorque(swigCPtr, ChVectorD.getCPtr(body_force), ChVectorD.getCPtr(body_torque));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void UpdateTime(double mytime) {
    corePINVOKE.ChForce_UpdateTime(swigCPtr, mytime);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void UpdateState() {
    corePINVOKE.ChForce_UpdateState(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Update(double mytime) {
    corePINVOKE.ChForce_Update(swigCPtr, mytime);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override void ArchiveOUT(SWIGTYPE_p_ChArchiveOut marchive) {
    corePINVOKE.ChForce_ArchiveOUT(swigCPtr, SWIGTYPE_p_ChArchiveOut.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override void ArchiveIN(SWIGTYPE_p_chrono__ChArchiveIn marchive) {
    corePINVOKE.ChForce_ArchiveIN(swigCPtr, SWIGTYPE_p_chrono__ChArchiveIn.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public enum ForceType {
    FORCE,
    TORQUE
  }

  public enum ReferenceFrame {
    BODY,
    WORLD
  }

  public enum AlignmentFrame {
    BODY_DIR,
    WORLD_DIR
  }

}
