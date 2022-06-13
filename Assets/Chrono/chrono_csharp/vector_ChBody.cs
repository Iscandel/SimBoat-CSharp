//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class vector_ChBody : global::System.IDisposable, global::System.Collections.IEnumerable, global::System.Collections.Generic.IEnumerable<ChBody>
 {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  protected bool swigCMemOwn;

  internal vector_ChBody(global::System.IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(vector_ChBody obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  ~vector_ChBody() {
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
          corePINVOKE.delete_vector_ChBody(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
    }
  }

  public vector_ChBody(global::System.Collections.IEnumerable c) : this() {
    if (c == null)
      throw new global::System.ArgumentNullException("c");
    foreach (ChBody element in c) {
      this.Add(element);
    }
  }

  public vector_ChBody(global::System.Collections.Generic.IEnumerable<ChBody> c) : this() {
    if (c == null)
      throw new global::System.ArgumentNullException("c");
    foreach (ChBody element in c) {
      this.Add(element);
    }
  }

  public bool IsFixedSize {
    get {
      return false;
    }
  }

  public bool IsReadOnly {
    get {
      return false;
    }
  }

  public ChBody this[int index]  {
    get {
      return getitem(index);
    }
    set {
      setitem(index, value);
    }
  }

  public int Capacity {
    get {
      return (int)capacity();
    }
    set {
      if (value < size())
        throw new global::System.ArgumentOutOfRangeException("Capacity");
      reserve((uint)value);
    }
  }

  public int Count {
    get {
      return (int)size();
    }
  }

  public bool IsSynchronized {
    get {
      return false;
    }
  }

  public void CopyTo(ChBody[] array)
  {
    CopyTo(0, array, 0, this.Count);
  }

  public void CopyTo(ChBody[] array, int arrayIndex)
  {
    CopyTo(0, array, arrayIndex, this.Count);
  }

  public void CopyTo(int index, ChBody[] array, int arrayIndex, int count)
  {
    if (array == null)
      throw new global::System.ArgumentNullException("array");
    if (index < 0)
      throw new global::System.ArgumentOutOfRangeException("index", "Value is less than zero");
    if (arrayIndex < 0)
      throw new global::System.ArgumentOutOfRangeException("arrayIndex", "Value is less than zero");
    if (count < 0)
      throw new global::System.ArgumentOutOfRangeException("count", "Value is less than zero");
    if (array.Rank > 1)
      throw new global::System.ArgumentException("Multi dimensional array.", "array");
    if (index+count > this.Count || arrayIndex+count > array.Length)
      throw new global::System.ArgumentException("Number of elements to copy is too large.");
    for (int i=0; i<count; i++)
      array.SetValue(getitemcopy(index+i), arrayIndex+i);
  }

  public ChBody[] ToArray() {
    ChBody[] array = new ChBody[this.Count];
    this.CopyTo(array);
    return array;
  }

  global::System.Collections.Generic.IEnumerator<ChBody> global::System.Collections.Generic.IEnumerable<ChBody>.GetEnumerator() {
    return new vector_ChBodyEnumerator(this);
  }

  global::System.Collections.IEnumerator global::System.Collections.IEnumerable.GetEnumerator() {
    return new vector_ChBodyEnumerator(this);
  }

  public vector_ChBodyEnumerator GetEnumerator() {
    return new vector_ChBodyEnumerator(this);
  }

  // Type-safe enumerator
  /// Note that the IEnumerator documentation requires an InvalidOperationException to be thrown
  /// whenever the collection is modified. This has been done for changes in the size of the
  /// collection but not when one of the elements of the collection is modified as it is a bit
  /// tricky to detect unmanaged code that modifies the collection under our feet.
  public sealed class vector_ChBodyEnumerator : global::System.Collections.IEnumerator
    , global::System.Collections.Generic.IEnumerator<ChBody>
  {
    private vector_ChBody collectionRef;
    private int currentIndex;
    private object currentObject;
    private int currentSize;

    public vector_ChBodyEnumerator(vector_ChBody collection) {
      collectionRef = collection;
      currentIndex = -1;
      currentObject = null;
      currentSize = collectionRef.Count;
    }

    // Type-safe iterator Current
    public ChBody Current {
      get {
        if (currentIndex == -1)
          throw new global::System.InvalidOperationException("Enumeration not started.");
        if (currentIndex > currentSize - 1)
          throw new global::System.InvalidOperationException("Enumeration finished.");
        if (currentObject == null)
          throw new global::System.InvalidOperationException("Collection modified.");
        return (ChBody)currentObject;
      }
    }

    // Type-unsafe IEnumerator.Current
    object global::System.Collections.IEnumerator.Current {
      get {
        return Current;
      }
    }

    public bool MoveNext() {
      int size = collectionRef.Count;
      bool moveOkay = (currentIndex+1 < size) && (size == currentSize);
      if (moveOkay) {
        currentIndex++;
        currentObject = collectionRef[currentIndex];
      } else {
        currentObject = null;
      }
      return moveOkay;
    }

    public void Reset() {
      currentIndex = -1;
      currentObject = null;
      if (collectionRef.Count != currentSize) {
        throw new global::System.InvalidOperationException("Collection modified.");
      }
    }

    public void Dispose() {
        currentIndex = -1;
        currentObject = null;
    }
  }

  public void Clear() {
    corePINVOKE.vector_ChBody_Clear(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Add(ChBody x) {
    corePINVOKE.vector_ChBody_Add(swigCPtr, ChBody.getCPtr(x));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  private uint size() {
    uint ret = corePINVOKE.vector_ChBody_size(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  private uint capacity() {
    uint ret = corePINVOKE.vector_ChBody_capacity(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  private void reserve(uint n) {
    corePINVOKE.vector_ChBody_reserve(swigCPtr, n);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public vector_ChBody() : this(corePINVOKE.new_vector_ChBody__SWIG_0(), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public vector_ChBody(vector_ChBody other) : this(corePINVOKE.new_vector_ChBody__SWIG_1(vector_ChBody.getCPtr(other)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public vector_ChBody(int capacity) : this(corePINVOKE.new_vector_ChBody__SWIG_2(capacity), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  private ChBody getitemcopy(int index) {
    global::System.IntPtr cPtr = corePINVOKE.vector_ChBody_getitemcopy(swigCPtr, index);
    ChBody ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChBody(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  private ChBody getitem(int index) {
    global::System.IntPtr cPtr = corePINVOKE.vector_ChBody_getitem(swigCPtr, index);
    ChBody ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChBody(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  private void setitem(int index, ChBody val) {
    corePINVOKE.vector_ChBody_setitem(swigCPtr, index, ChBody.getCPtr(val));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void AddRange(vector_ChBody values) {
    corePINVOKE.vector_ChBody_AddRange(swigCPtr, vector_ChBody.getCPtr(values));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public vector_ChBody GetRange(int index, int count) {
    global::System.IntPtr cPtr = corePINVOKE.vector_ChBody_GetRange(swigCPtr, index, count);
    vector_ChBody ret = (cPtr == global::System.IntPtr.Zero) ? null : new vector_ChBody(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void Insert(int index, ChBody x) {
    corePINVOKE.vector_ChBody_Insert(swigCPtr, index, ChBody.getCPtr(x));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void InsertRange(int index, vector_ChBody values) {
    corePINVOKE.vector_ChBody_InsertRange(swigCPtr, index, vector_ChBody.getCPtr(values));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void RemoveAt(int index) {
    corePINVOKE.vector_ChBody_RemoveAt(swigCPtr, index);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void RemoveRange(int index, int count) {
    corePINVOKE.vector_ChBody_RemoveRange(swigCPtr, index, count);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public static vector_ChBody Repeat(ChBody value, int count) {
    global::System.IntPtr cPtr = corePINVOKE.vector_ChBody_Repeat(ChBody.getCPtr(value), count);
    vector_ChBody ret = (cPtr == global::System.IntPtr.Zero) ? null : new vector_ChBody(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void Reverse() {
    corePINVOKE.vector_ChBody_Reverse__SWIG_0(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void Reverse(int index, int count) {
    corePINVOKE.vector_ChBody_Reverse__SWIG_1(swigCPtr, index, count);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public void SetRange(int index, vector_ChBody values) {
    corePINVOKE.vector_ChBody_SetRange(swigCPtr, index, vector_ChBody.getCPtr(values));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

}
