using DDS;
using DDS.OpenSplice;

namespace chat
{
    #region IChatMsgDataReader
    public interface IChatMsgDataReader : DDS.IDataReader
    {
        ReturnCode Read(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos);

        ReturnCode Read(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples);

        ReturnCode Read(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode Read(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode Take(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos);

        ReturnCode Take(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples);

        ReturnCode Take(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode Take(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode ReadWithCondition(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition);

        ReturnCode ReadWithCondition(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            IReadCondition readCondition);

        ReturnCode TakeWithCondition(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition);

        ReturnCode TakeWithCondition(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            IReadCondition readCondition);

        ReturnCode ReadNextSample(
            ref ChatMsg dataValue,
            ref SampleInfo sampleInfo);

        ReturnCode TakeNextSample(
            ref ChatMsg dataValue,
            ref SampleInfo sampleInfo);

        ReturnCode ReadInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode ReadInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode ReadInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode TakeInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode TakeInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode TakeInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode ReadNextInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode ReadNextInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode ReadNextInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode TakeNextInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle);

        ReturnCode TakeNextInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle);

        ReturnCode TakeNextInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates);

        ReturnCode ReadNextInstanceWithCondition(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode ReadNextInstanceWithCondition(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode TakeNextInstanceWithCondition(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode TakeNextInstanceWithCondition(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle,
            IReadCondition readCondition);

        ReturnCode ReturnLoan(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos);

        ReturnCode GetKeyValue(
            ref ChatMsg key,
            InstanceHandle handle);

        InstanceHandle LookupInstance(
            ChatMsg instance);
    }
    #endregion

    #region IChatMsgDataWriter
    public interface IChatMsgDataWriter : DDS.IDataWriter
    {
        InstanceHandle RegisterInstance(
            ChatMsg instanceData);

        InstanceHandle RegisterInstanceWithTimestamp(
            ChatMsg instanceData,
            Time sourceTimestamp);

        ReturnCode UnregisterInstance(
            ChatMsg instanceData,
            InstanceHandle instanceHandle);

        ReturnCode UnregisterInstanceWithTimestamp(
            ChatMsg instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode Write(
            ChatMsg instanceData);

        ReturnCode Write(
            ChatMsg instanceData,
            InstanceHandle instanceHandle);

        ReturnCode WriteWithTimestamp(
            ChatMsg instanceData,
            Time sourceTimestamp);

        ReturnCode WriteWithTimestamp(
            ChatMsg instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode Dispose(
            ChatMsg instanceData,
            InstanceHandle instanceHandle);

        ReturnCode DisposeWithTimestamp(
            ChatMsg instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode WriteDispose(
            ChatMsg instanceData);

        ReturnCode WriteDispose(
            ChatMsg instanceData,
            InstanceHandle instanceHandle);

        ReturnCode WriteDisposeWithTimestamp(
            ChatMsg instanceData,
            Time sourceTimestamp);

        ReturnCode WriteDisposeWithTimestamp(
            ChatMsg instanceData,
            InstanceHandle instanceHandle,
            Time sourceTimestamp);

        ReturnCode GetKeyValue(
            ref ChatMsg key,
            InstanceHandle instanceHandle);

        InstanceHandle LookupInstance(
            ChatMsg instanceData);
    }
    #endregion

}

