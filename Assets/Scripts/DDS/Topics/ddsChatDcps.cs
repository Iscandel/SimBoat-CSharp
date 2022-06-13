using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using DDS;
using DDS.OpenSplice;
using DDS.OpenSplice.CustomMarshalers;

namespace chat
{
    #region ChatMsgDataReader
    public class ChatMsgDataReader : DDS.OpenSplice.FooDataReader<ChatMsg, __ChatMsgMarshaler>, 
                                         IChatMsgDataReader
    {
        public ChatMsgDataReader(DatabaseMarshaler marshaler)
            : base(marshaler) { }

        public ReturnCode Read(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos)
        {
            return Read(ref dataValues, ref sampleInfos, Length.Unlimited);
        }

        public ReturnCode Read(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples)
        {
            return Read(ref dataValues, ref sampleInfos, maxSamples, SampleStateKind.Any,
                ViewStateKind.Any, InstanceStateKind.Any);
        }

        public ReturnCode Read(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates)
        {
            return Read(ref dataValues, ref sampleInfos, Length.Unlimited, sampleStates,
                viewStates, instanceStates);
        }

        public override ReturnCode Read(
                ref ChatMsg[] dataValues,
                ref SampleInfo[] sampleInfos,
                int maxSamples,
                SampleStateKind sampleStates,
                ViewStateKind viewStates,
                InstanceStateKind instanceStates)
        {
            ReturnCode result =
                base.Read(
                        ref dataValues,
                        ref sampleInfos,
                        maxSamples,
                        sampleStates,
                        viewStates,
                        instanceStates);
            return result;
        }

        public ReturnCode Take(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos)
        {
            return Take(ref dataValues, ref sampleInfos, Length.Unlimited);
        }

        public ReturnCode Take(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples)
        {
            return Take(ref dataValues, ref sampleInfos, maxSamples, SampleStateKind.Any,
                ViewStateKind.Any, InstanceStateKind.Any);
        }

        public ReturnCode Take(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates)
        {
            return Take(ref dataValues, ref sampleInfos, Length.Unlimited, sampleStates,
                viewStates, instanceStates);
        }

        public override ReturnCode Take(
                ref ChatMsg[] dataValues,
                ref SampleInfo[] sampleInfos,
                int maxSamples,
                SampleStateKind sampleStates,
                ViewStateKind viewStates,
                InstanceStateKind instanceStates)
        {
            ReturnCode result =
                base.Take(
                        ref dataValues,
                        ref sampleInfos,
                        maxSamples,
                        sampleStates,
                        viewStates,
                        instanceStates);
            return result;
        }

        public ReturnCode ReadWithCondition(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition)
        {
            return ReadWithCondition(ref dataValues, ref sampleInfos,
                Length.Unlimited, readCondition);
        }

        public override ReturnCode ReadWithCondition(
                ref ChatMsg[] dataValues,
                ref SampleInfo[] sampleInfos,
                int maxSamples,
                IReadCondition readCondition)
        {
            ReturnCode result =
                base.ReadWithCondition(
                        ref dataValues,
                        ref sampleInfos,
                        maxSamples,
                        readCondition);
            return result;
        }

        public ReturnCode TakeWithCondition(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition)
        {
            return TakeWithCondition(ref dataValues, ref sampleInfos,
                Length.Unlimited, readCondition);
        }

        public override ReturnCode TakeWithCondition(
                ref ChatMsg[] dataValues,
                ref SampleInfo[] sampleInfos,
                int maxSamples,
                IReadCondition readCondition)
        {
            ReturnCode result =
                base.TakeWithCondition(
                        ref dataValues,
                        ref sampleInfos,
                        maxSamples,
                        readCondition);
            return result;
        }

        public override ReturnCode ReadNextSample(
                ref ChatMsg dataValue,
                ref SampleInfo sampleInfo)
        {
            ReturnCode result =
                base.ReadNextSample(
                        ref dataValue,
                        ref sampleInfo);
            return result;
        }

        public override ReturnCode TakeNextSample(
                ref ChatMsg dataValue,
                ref SampleInfo sampleInfo)
        {
            ReturnCode result =
                base.TakeNextSample(
                        ref dataValue,
                        ref sampleInfo);
            return result;
        }

        public ReturnCode ReadInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return ReadInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode ReadInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return ReadInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode ReadInstance(
                ref ChatMsg[] dataValues,
                ref SampleInfo[] sampleInfos,
                int maxSamples,
                InstanceHandle instanceHandle,
                SampleStateKind sampleStates,
                ViewStateKind viewStates,
                InstanceStateKind instanceStates)
        {
            ReturnCode result =
                base.ReadInstance(
                        ref dataValues,
                        ref sampleInfos,
                        maxSamples,
                        instanceHandle,
                        sampleStates,
                        viewStates,
                        instanceStates);
            return result;
        }

        public ReturnCode TakeInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return TakeInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode TakeInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return TakeInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode TakeInstance(
                ref ChatMsg[] dataValues,
                ref SampleInfo[] sampleInfos,
                int maxSamples,
                InstanceHandle instanceHandle,
                SampleStateKind sampleStates,
                ViewStateKind viewStates,
                InstanceStateKind instanceStates)
        {
            ReturnCode result =
                base.TakeInstance(
                        ref dataValues,
                        ref sampleInfos,
                        maxSamples,
                        instanceHandle,
                        sampleStates,
                        viewStates,
                        instanceStates);
            return result;
        }

        public ReturnCode ReadNextInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return ReadNextInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode ReadNextInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return ReadNextInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode ReadNextInstance(
                ref ChatMsg[] dataValues,
                ref SampleInfo[] sampleInfos,
                int maxSamples,
                InstanceHandle instanceHandle,
                SampleStateKind sampleStates,
                ViewStateKind viewStates,
                InstanceStateKind instanceStates)
        {
            ReturnCode result =
                base.ReadNextInstance(
                        ref dataValues,
                        ref sampleInfos,
                        maxSamples,
                        instanceHandle,
                        sampleStates,
                        viewStates,
                        instanceStates);
            return result;
        }

        public ReturnCode TakeNextInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return TakeNextInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode TakeNextInstance(
            ref ChatMsg[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return TakeNextInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode TakeNextInstance(
                ref ChatMsg[] dataValues,
                ref SampleInfo[] sampleInfos,
                int maxSamples,
                InstanceHandle instanceHandle,
                SampleStateKind sampleStates,
                ViewStateKind viewStates,
                InstanceStateKind instanceStates)
        {
            ReturnCode result =
                base.TakeNextInstance(
                        ref dataValues,
                        ref sampleInfos,
                        maxSamples,
                        instanceHandle,
                        sampleStates,
                        viewStates,
                        instanceStates);
            return result;
        }

        public ReturnCode ReadNextInstanceWithCondition(
                ref ChatMsg[] dataValues,
                ref SampleInfo[] sampleInfos,
                InstanceHandle instanceHandle,
                IReadCondition readCondition)
        {
            return ReadNextInstanceWithCondition(
                ref dataValues,
                ref sampleInfos,
                Length.Unlimited,
                instanceHandle,
                readCondition);
        }

        public override ReturnCode ReadNextInstanceWithCondition(
                ref ChatMsg[] dataValues,
                ref SampleInfo[] sampleInfos,
                int maxSamples,
                InstanceHandle instanceHandle,
                IReadCondition readCondition)
        {
            ReturnCode result =
                base.ReadNextInstanceWithCondition(
                        ref dataValues,
                        ref sampleInfos,
                        maxSamples,
                        instanceHandle,
                        readCondition);
            return result;
        }

        public ReturnCode TakeNextInstanceWithCondition(
                ref ChatMsg[] dataValues,
                ref SampleInfo[] sampleInfos,
                InstanceHandle instanceHandle,
                IReadCondition readCondition)
        {
            return TakeNextInstanceWithCondition(
                ref dataValues,
                ref sampleInfos,
                Length.Unlimited,
                instanceHandle,
                readCondition);
        }

        public override ReturnCode TakeNextInstanceWithCondition(
                ref ChatMsg[] dataValues,
                ref SampleInfo[] sampleInfos,
                int maxSamples,
                InstanceHandle instanceHandle,
                IReadCondition readCondition)
        {
            ReturnCode result =
                base.TakeNextInstanceWithCondition(
                        ref dataValues,
                        ref sampleInfos,
                        maxSamples,
                        instanceHandle,
                        readCondition);

            return result;
        }

        public override ReturnCode ReturnLoan(
                ref ChatMsg[] dataValues,
                ref SampleInfo[] sampleInfos)
        {
            ReturnCode result =
                base.ReturnLoan(
                        ref dataValues,
                        ref sampleInfos);

            return result;
        }

        public override ReturnCode GetKeyValue(
                ref ChatMsg key,
                InstanceHandle handle)
        {
            ReturnCode result = base.GetKeyValue(
                        ref key,
                        handle);
            return result;
        }

        public override InstanceHandle LookupInstance(
                ChatMsg instance)
        {
            return
                base.LookupInstance(
                        instance);
        }

    }
    #endregion
    
    #region ChatMsgDataWriter
    public class ChatMsgDataWriter : DDS.OpenSplice.FooDataWriter<ChatMsg, __ChatMsgMarshaler>, 
                                         IChatMsgDataWriter
    {
        public ChatMsgDataWriter(DatabaseMarshaler marshaler)
            : base(marshaler) { }

        public InstanceHandle RegisterInstance(
                ChatMsg instanceData)
        {
            return base.RegisterInstance(
                    instanceData,
                    Time.Current);
        }

        public InstanceHandle RegisterInstanceWithTimestamp(
                ChatMsg instanceData,
                Time sourceTimestamp)
        {
            return base.RegisterInstance(
                    instanceData,
                    sourceTimestamp);
        }

        public ReturnCode UnregisterInstance(
                ChatMsg instanceData,
                InstanceHandle instanceHandle)
        {
            return base.UnregisterInstance(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode UnregisterInstanceWithTimestamp(
                ChatMsg instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.UnregisterInstance(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public ReturnCode Write(ChatMsg instanceData)
        {
            return base.Write(
                    instanceData,
                    InstanceHandle.Nil,
                    Time.Current);
        }

        public ReturnCode Write(
                ChatMsg instanceData,
                InstanceHandle instanceHandle)
        {
            return base.Write(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode WriteWithTimestamp(
                ChatMsg instanceData,
                Time sourceTimestamp)
        {
            return base.Write(
                    instanceData,
                    InstanceHandle.Nil,
                    sourceTimestamp);
        }

        public ReturnCode WriteWithTimestamp(
                ChatMsg instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.Write(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public ReturnCode Dispose(
                ChatMsg instanceData,
                InstanceHandle instanceHandle)
        {
            return base.Dispose(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode DisposeWithTimestamp(
                ChatMsg instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.Dispose(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public ReturnCode WriteDispose(
                ChatMsg instanceData)
        {
            return base.WriteDispose(
                    instanceData,
                    InstanceHandle.Nil,
                    Time.Current);
        }

        public ReturnCode WriteDispose(
                ChatMsg instanceData,
                InstanceHandle instanceHandle)
        {
            return base.WriteDispose(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode WriteDisposeWithTimestamp(
                ChatMsg instanceData,
                Time sourceTimestamp)
        {
            return base.WriteDispose(
                    instanceData,
                    InstanceHandle.Nil,
                    sourceTimestamp);
        }

        public ReturnCode WriteDisposeWithTimestamp(
                ChatMsg instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.WriteDispose(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public override ReturnCode GetKeyValue(
                ref ChatMsg key,
                InstanceHandle instanceHandle)
        {
            return base.GetKeyValue(ref key, instanceHandle);
        }

        public override InstanceHandle LookupInstance(
            ChatMsg instanceData)
        {
            return base.LookupInstance(instanceData);
        }
    }
    #endregion

    #region ChatMsgTypeSupport
    public class ChatMsgTypeSupport : DDS.OpenSplice.TypeSupport
    {
        private static readonly string[] metaDescriptor = {"<MetaData version=\"1.0.0\"><Module name=\"chat\"><Struct name=\"ChatMsg\"><Member name=\"id\"><Short/>",
"</Member><Member name=\"cpt\"><Long/></Member><Member name=\"text\"><String/></Member></Struct></Module>",
"</MetaData>"};

        public ChatMsgTypeSupport()
            : base(typeof(ChatMsg), metaDescriptor, "chat::ChatMsg", "", "id")
        { }


        public override ReturnCode RegisterType(IDomainParticipant participant, string typeName)
        {
            return RegisterType(participant, typeName, new __ChatMsgMarshaler());
        }

        public override DDS.OpenSplice.DataWriter CreateDataWriter(DatabaseMarshaler marshaler)
        {
            return new ChatMsgDataWriter(marshaler);
        }

        public override DDS.OpenSplice.DataReader CreateDataReader(DatabaseMarshaler marshaler)
        {
            return new ChatMsgDataReader(marshaler);
        }
    }
    #endregion

}

