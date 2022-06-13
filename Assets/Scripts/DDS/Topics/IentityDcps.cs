using DDS;
using DDS.OpenSplice;

namespace entity
{
    #region IEntityInfoDataReader
    public interface IEntityInfoDataReader : DDS.IDataReader
    {
        ReturnCode Read(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos);

        ReturnCode Read(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples);

        ReturnCode Read(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode Read(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode Take(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos);

        ReturnCode Take(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples);

        ReturnCode Take(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode Take(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode ReadWithCondition(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition);

        ReturnCode ReadWithCondition(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            IReadCondition readCondition);

        ReturnCode TakeWithCondition(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition);

        ReturnCode TakeWithCondition(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            IReadCondition readCondition);

        ReturnCode ReadNextSample(
            ref EntityInfo dataValue,
            ref SampleInfo sampleInfo);

        ReturnCode TakeNextSample(
            ref EntityInfo dataValue,
            ref SampleInfo sampleInfo);

        ReturnCode ReadInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode ReadInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode ReadInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode TakeInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode TakeInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode TakeInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode ReadNextInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode ReadNextInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode ReadNextInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode TakeNextInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode TakeNextInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode TakeNextInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode ReadNextInstanceWithCondition(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode ReadNextInstanceWithCondition(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode TakeNextInstanceWithCondition(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode TakeNextInstanceWithCondition(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode ReturnLoan(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos);

        ReturnCode GetKeyValue(
            ref EntityInfo key,
            InstanceHandle handle);

        InstanceHandle LookupInstance(
            EntityInfo instance);
    }
    #endregion

    #region IEntityInfoDataWriter
    public interface IEntityInfoDataWriter : DDS.IDataWriter
    {
        InstanceHandle RegisterInstance(
            EntityInfo instanceData);

        InstanceHandle RegisterInstanceWithTimestamp(
            EntityInfo instanceData,
            Time sourceTimestamp);

        ReturnCode UnregisterInstance(
            EntityInfo instanceData,
            InstanceHandle instanceHandle);

        ReturnCode UnregisterInstanceWithTimestamp(
            EntityInfo instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode Write(
            EntityInfo instanceData);

        ReturnCode Write(
            EntityInfo instanceData,
            InstanceHandle instanceHandle);

        ReturnCode WriteWithTimestamp(
            EntityInfo instanceData,
            Time sourceTimestamp);

        ReturnCode WriteWithTimestamp(
            EntityInfo instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode Dispose(
            EntityInfo instanceData,
            InstanceHandle instanceHandle);

        ReturnCode DisposeWithTimestamp(
            EntityInfo instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode WriteDispose(
            EntityInfo instanceData);

        ReturnCode WriteDispose(
            EntityInfo instanceData,
            InstanceHandle instanceHandle);

        ReturnCode WriteDisposeWithTimestamp(
            EntityInfo instanceData,
            Time sourceTimestamp);

        ReturnCode WriteDisposeWithTimestamp(
            EntityInfo instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode GetKeyValue(
            ref EntityInfo key,
            InstanceHandle instanceHandle);

        InstanceHandle LookupInstance(
            EntityInfo instanceData);
    }
    #endregion

    #region IEntityKinematicsInfoDataReader
    public interface IEntityKinematicsInfoDataReader : DDS.IDataReader
    {
        ReturnCode Read(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos);

        ReturnCode Read(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples);

        ReturnCode Read(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode Read(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode Take(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos);

        ReturnCode Take(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples);

        ReturnCode Take(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode Take(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode ReadWithCondition(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition);

        ReturnCode ReadWithCondition(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            IReadCondition readCondition);

        ReturnCode TakeWithCondition(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition);

        ReturnCode TakeWithCondition(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            IReadCondition readCondition);

        ReturnCode ReadNextSample(
            ref EntityKinematicsInfo dataValue,
            ref SampleInfo sampleInfo);

        ReturnCode TakeNextSample(
            ref EntityKinematicsInfo dataValue,
            ref SampleInfo sampleInfo);

        ReturnCode ReadInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode ReadInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode ReadInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode TakeInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode TakeInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode TakeInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode ReadNextInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode ReadNextInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode ReadNextInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode TakeNextInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode TakeNextInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode TakeNextInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode ReadNextInstanceWithCondition(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode ReadNextInstanceWithCondition(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode TakeNextInstanceWithCondition(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode TakeNextInstanceWithCondition(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode ReturnLoan(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos);

        ReturnCode GetKeyValue(
            ref EntityKinematicsInfo key,
            InstanceHandle handle);

        InstanceHandle LookupInstance(
            EntityKinematicsInfo instance);
    }
    #endregion

    #region IEntityKinematicsInfoDataWriter
    public interface IEntityKinematicsInfoDataWriter : DDS.IDataWriter
    {
        InstanceHandle RegisterInstance(
            EntityKinematicsInfo instanceData);

        InstanceHandle RegisterInstanceWithTimestamp(
            EntityKinematicsInfo instanceData,
            Time sourceTimestamp);

        ReturnCode UnregisterInstance(
            EntityKinematicsInfo instanceData,
            InstanceHandle instanceHandle);

        ReturnCode UnregisterInstanceWithTimestamp(
            EntityKinematicsInfo instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode Write(
            EntityKinematicsInfo instanceData);

        ReturnCode Write(
            EntityKinematicsInfo instanceData,
            InstanceHandle instanceHandle);

        ReturnCode WriteWithTimestamp(
            EntityKinematicsInfo instanceData,
            Time sourceTimestamp);

        ReturnCode WriteWithTimestamp(
            EntityKinematicsInfo instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode Dispose(
            EntityKinematicsInfo instanceData,
            InstanceHandle instanceHandle);

        ReturnCode DisposeWithTimestamp(
            EntityKinematicsInfo instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode WriteDispose(
            EntityKinematicsInfo instanceData);

        ReturnCode WriteDispose(
            EntityKinematicsInfo instanceData,
            InstanceHandle instanceHandle);

        ReturnCode WriteDisposeWithTimestamp(
            EntityKinematicsInfo instanceData,
            Time sourceTimestamp);

        ReturnCode WriteDisposeWithTimestamp(
            EntityKinematicsInfo instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode GetKeyValue(
            ref EntityKinematicsInfo key,
            InstanceHandle instanceHandle);

        InstanceHandle LookupInstance(
            EntityKinematicsInfo instanceData);
    }
    #endregion

    #region IEntityShapeDataReader
    public interface IEntityShapeDataReader : DDS.IDataReader
    {
        ReturnCode Read(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos);

        ReturnCode Read(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples);

        ReturnCode Read(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode Read(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode Take(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos);

        ReturnCode Take(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples);

        ReturnCode Take(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode Take(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode ReadWithCondition(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition);

        ReturnCode ReadWithCondition(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            IReadCondition readCondition);

        ReturnCode TakeWithCondition(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition);

        ReturnCode TakeWithCondition(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            IReadCondition readCondition);

        ReturnCode ReadNextSample(
            ref EntityShape dataValue,
            ref SampleInfo sampleInfo);

        ReturnCode TakeNextSample(
            ref EntityShape dataValue,
            ref SampleInfo sampleInfo);

        ReturnCode ReadInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode ReadInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode ReadInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode TakeInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode TakeInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode TakeInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode ReadNextInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode ReadNextInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode ReadNextInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode TakeNextInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode TakeNextInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode TakeNextInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode ReadNextInstanceWithCondition(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode ReadNextInstanceWithCondition(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode TakeNextInstanceWithCondition(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode TakeNextInstanceWithCondition(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode ReturnLoan(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos);

        ReturnCode GetKeyValue(
            ref EntityShape key,
            InstanceHandle handle);

        InstanceHandle LookupInstance(
            EntityShape instance);
    }
    #endregion

    #region IEntityShapeDataWriter
    public interface IEntityShapeDataWriter : DDS.IDataWriter
    {
        InstanceHandle RegisterInstance(
            EntityShape instanceData);

        InstanceHandle RegisterInstanceWithTimestamp(
            EntityShape instanceData,
            Time sourceTimestamp);

        ReturnCode UnregisterInstance(
            EntityShape instanceData,
            InstanceHandle instanceHandle);

        ReturnCode UnregisterInstanceWithTimestamp(
            EntityShape instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode Write(
            EntityShape instanceData);

        ReturnCode Write(
            EntityShape instanceData,
            InstanceHandle instanceHandle);

        ReturnCode WriteWithTimestamp(
            EntityShape instanceData,
            Time sourceTimestamp);

        ReturnCode WriteWithTimestamp(
            EntityShape instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode Dispose(
            EntityShape instanceData,
            InstanceHandle instanceHandle);

        ReturnCode DisposeWithTimestamp(
            EntityShape instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode WriteDispose(
            EntityShape instanceData);

        ReturnCode WriteDispose(
            EntityShape instanceData,
            InstanceHandle instanceHandle);

        ReturnCode WriteDisposeWithTimestamp(
            EntityShape instanceData,
            Time sourceTimestamp);

        ReturnCode WriteDisposeWithTimestamp(
            EntityShape instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode GetKeyValue(
            ref EntityShape key,
            InstanceHandle instanceHandle);

        InstanceHandle LookupInstance(
            EntityShape instanceData);
    }
    #endregion

}

