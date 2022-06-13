using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using DDS;
using DDS.OpenSplice;
using DDS.OpenSplice.CustomMarshalers;

namespace entity
{
    #region EntityInfoDataReader
    public class EntityInfoDataReader : DDS.OpenSplice.FooDataReader<EntityInfo, __EntityInfoMarshaler>, 
                                         IEntityInfoDataReader
    {
        public EntityInfoDataReader(DatabaseMarshaler marshaler)
            : base(marshaler) { }

        public ReturnCode Read(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos)
        {
            return Read(ref dataValues, ref sampleInfos, Length.Unlimited);
        }

        public ReturnCode Read(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples)
        {
            return Read(ref dataValues, ref sampleInfos, maxSamples, SampleStateKind.Any,
                ViewStateKind.Any, InstanceStateKind.Any);
        }

        public ReturnCode Read(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates)
        {
            return Read(ref dataValues, ref sampleInfos, Length.Unlimited, sampleStates,
                viewStates, instanceStates);
        }

        public override ReturnCode Read(
                ref EntityInfo[] dataValues,
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
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos)
        {
            return Take(ref dataValues, ref sampleInfos, Length.Unlimited);
        }

        public ReturnCode Take(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples)
        {
            return Take(ref dataValues, ref sampleInfos, maxSamples, SampleStateKind.Any,
                ViewStateKind.Any, InstanceStateKind.Any);
        }

        public ReturnCode Take(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates)
        {
            return Take(ref dataValues, ref sampleInfos, Length.Unlimited, sampleStates,
                viewStates, instanceStates);
        }

        public override ReturnCode Take(
                ref EntityInfo[] dataValues,
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
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition)
        {
            return ReadWithCondition(ref dataValues, ref sampleInfos,
                Length.Unlimited, readCondition);
        }

        public override ReturnCode ReadWithCondition(
                ref EntityInfo[] dataValues,
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
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition)
        {
            return TakeWithCondition(ref dataValues, ref sampleInfos,
                Length.Unlimited, readCondition);
        }

        public override ReturnCode TakeWithCondition(
                ref EntityInfo[] dataValues,
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
                ref EntityInfo dataValue,
                ref SampleInfo sampleInfo)
        {
            ReturnCode result =
                base.ReadNextSample(
                        ref dataValue,
                        ref sampleInfo);
            return result;
        }

        public override ReturnCode TakeNextSample(
                ref EntityInfo dataValue,
                ref SampleInfo sampleInfo)
        {
            ReturnCode result =
                base.TakeNextSample(
                        ref dataValue,
                        ref sampleInfo);
            return result;
        }

        public ReturnCode ReadInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return ReadInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode ReadInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return ReadInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode ReadInstance(
                ref EntityInfo[] dataValues,
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
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return TakeInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode TakeInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return TakeInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode TakeInstance(
                ref EntityInfo[] dataValues,
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
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return ReadNextInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode ReadNextInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return ReadNextInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode ReadNextInstance(
                ref EntityInfo[] dataValues,
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
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return TakeNextInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode TakeNextInstance(
            ref EntityInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return TakeNextInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode TakeNextInstance(
                ref EntityInfo[] dataValues,
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
                ref EntityInfo[] dataValues,
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
                ref EntityInfo[] dataValues,
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
                ref EntityInfo[] dataValues,
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
                ref EntityInfo[] dataValues,
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
                ref EntityInfo[] dataValues,
                ref SampleInfo[] sampleInfos)
        {
            ReturnCode result =
                base.ReturnLoan(
                        ref dataValues,
                        ref sampleInfos);

            return result;
        }

        public override ReturnCode GetKeyValue(
                ref EntityInfo key,
                InstanceHandle handle)
        {
            ReturnCode result = base.GetKeyValue(
                        ref key,
                        handle);
            return result;
        }

        public override InstanceHandle LookupInstance(
                EntityInfo instance)
        {
            return
                base.LookupInstance(
                        instance);
        }

    }
    #endregion
    
    #region EntityInfoDataWriter
    public class EntityInfoDataWriter : DDS.OpenSplice.FooDataWriter<EntityInfo, __EntityInfoMarshaler>, 
                                         IEntityInfoDataWriter
    {
        public EntityInfoDataWriter(DatabaseMarshaler marshaler)
            : base(marshaler) { }

        public InstanceHandle RegisterInstance(
                EntityInfo instanceData)
        {
            return base.RegisterInstance(
                    instanceData,
                    Time.Current);
        }

        public InstanceHandle RegisterInstanceWithTimestamp(
                EntityInfo instanceData,
                Time sourceTimestamp)
        {
            return base.RegisterInstance(
                    instanceData,
                    sourceTimestamp);
        }

        public ReturnCode UnregisterInstance(
                EntityInfo instanceData,
                InstanceHandle instanceHandle)
        {
            return base.UnregisterInstance(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode UnregisterInstanceWithTimestamp(
                EntityInfo instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.UnregisterInstance(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public ReturnCode Write(EntityInfo instanceData)
        {
            return base.Write(
                    instanceData,
                    InstanceHandle.Nil,
                    Time.Current);
        }

        public ReturnCode Write(
                EntityInfo instanceData,
                InstanceHandle instanceHandle)
        {
            return base.Write(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode WriteWithTimestamp(
                EntityInfo instanceData,
                Time sourceTimestamp)
        {
            return base.Write(
                    instanceData,
                    InstanceHandle.Nil,
                    sourceTimestamp);
        }

        public ReturnCode WriteWithTimestamp(
                EntityInfo instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.Write(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public ReturnCode Dispose(
                EntityInfo instanceData,
                InstanceHandle instanceHandle)
        {
            return base.Dispose(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode DisposeWithTimestamp(
                EntityInfo instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.Dispose(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public ReturnCode WriteDispose(
                EntityInfo instanceData)
        {
            return base.WriteDispose(
                    instanceData,
                    InstanceHandle.Nil,
                    Time.Current);
        }

        public ReturnCode WriteDispose(
                EntityInfo instanceData,
                InstanceHandle instanceHandle)
        {
            return base.WriteDispose(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode WriteDisposeWithTimestamp(
                EntityInfo instanceData,
                Time sourceTimestamp)
        {
            return base.WriteDispose(
                    instanceData,
                    InstanceHandle.Nil,
                    sourceTimestamp);
        }

        public ReturnCode WriteDisposeWithTimestamp(
                EntityInfo instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.WriteDispose(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public override ReturnCode GetKeyValue(
                ref EntityInfo key,
                InstanceHandle instanceHandle)
        {
            return base.GetKeyValue(ref key, instanceHandle);
        }

        public override InstanceHandle LookupInstance(
            EntityInfo instanceData)
        {
            return base.LookupInstance(instanceData);
        }
    }
    #endregion

    #region EntityInfoTypeSupport
    public class EntityInfoTypeSupport : DDS.OpenSplice.TypeSupport
    {
        private static readonly string[] metaDescriptor = {"<MetaData version=\"1.0.0\"><Module name=\"entity\"><Struct name=\"EntityInfo\"><Member name=\"id\"><Short/>",
"</Member><Member name=\"type\"><String/></Member></Struct></Module></MetaData>"};

        public EntityInfoTypeSupport()
            : base(typeof(EntityInfo), metaDescriptor, "entity::EntityInfo", "", "id")
        { }


        public override ReturnCode RegisterType(IDomainParticipant participant, string typeName)
        {
            return RegisterType(participant, typeName, new __EntityInfoMarshaler());
        }

        public override DDS.OpenSplice.DataWriter CreateDataWriter(DatabaseMarshaler marshaler)
        {
            return new EntityInfoDataWriter(marshaler);
        }

        public override DDS.OpenSplice.DataReader CreateDataReader(DatabaseMarshaler marshaler)
        {
            return new EntityInfoDataReader(marshaler);
        }
    }
    #endregion

    #region EntityKinematicsInfoDataReader
    public class EntityKinematicsInfoDataReader : DDS.OpenSplice.FooDataReader<EntityKinematicsInfo, __EntityKinematicsInfoMarshaler>, 
                                         IEntityKinematicsInfoDataReader
    {
        public EntityKinematicsInfoDataReader(DatabaseMarshaler marshaler)
            : base(marshaler) { }

        public ReturnCode Read(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos)
        {
            return Read(ref dataValues, ref sampleInfos, Length.Unlimited);
        }

        public ReturnCode Read(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples)
        {
            return Read(ref dataValues, ref sampleInfos, maxSamples, SampleStateKind.Any,
                ViewStateKind.Any, InstanceStateKind.Any);
        }

        public ReturnCode Read(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates)
        {
            return Read(ref dataValues, ref sampleInfos, Length.Unlimited, sampleStates,
                viewStates, instanceStates);
        }

        public override ReturnCode Read(
                ref EntityKinematicsInfo[] dataValues,
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
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos)
        {
            return Take(ref dataValues, ref sampleInfos, Length.Unlimited);
        }

        public ReturnCode Take(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples)
        {
            return Take(ref dataValues, ref sampleInfos, maxSamples, SampleStateKind.Any,
                ViewStateKind.Any, InstanceStateKind.Any);
        }

        public ReturnCode Take(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates)
        {
            return Take(ref dataValues, ref sampleInfos, Length.Unlimited, sampleStates,
                viewStates, instanceStates);
        }

        public override ReturnCode Take(
                ref EntityKinematicsInfo[] dataValues,
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
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition)
        {
            return ReadWithCondition(ref dataValues, ref sampleInfos,
                Length.Unlimited, readCondition);
        }

        public override ReturnCode ReadWithCondition(
                ref EntityKinematicsInfo[] dataValues,
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
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition)
        {
            return TakeWithCondition(ref dataValues, ref sampleInfos,
                Length.Unlimited, readCondition);
        }

        public override ReturnCode TakeWithCondition(
                ref EntityKinematicsInfo[] dataValues,
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
                ref EntityKinematicsInfo dataValue,
                ref SampleInfo sampleInfo)
        {
            ReturnCode result =
                base.ReadNextSample(
                        ref dataValue,
                        ref sampleInfo);
            return result;
        }

        public override ReturnCode TakeNextSample(
                ref EntityKinematicsInfo dataValue,
                ref SampleInfo sampleInfo)
        {
            ReturnCode result =
                base.TakeNextSample(
                        ref dataValue,
                        ref sampleInfo);
            return result;
        }

        public ReturnCode ReadInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return ReadInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode ReadInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return ReadInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode ReadInstance(
                ref EntityKinematicsInfo[] dataValues,
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
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return TakeInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode TakeInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return TakeInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode TakeInstance(
                ref EntityKinematicsInfo[] dataValues,
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
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return ReadNextInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode ReadNextInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return ReadNextInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode ReadNextInstance(
                ref EntityKinematicsInfo[] dataValues,
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
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return TakeNextInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode TakeNextInstance(
            ref EntityKinematicsInfo[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return TakeNextInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode TakeNextInstance(
                ref EntityKinematicsInfo[] dataValues,
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
                ref EntityKinematicsInfo[] dataValues,
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
                ref EntityKinematicsInfo[] dataValues,
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
                ref EntityKinematicsInfo[] dataValues,
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
                ref EntityKinematicsInfo[] dataValues,
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
                ref EntityKinematicsInfo[] dataValues,
                ref SampleInfo[] sampleInfos)
        {
            ReturnCode result =
                base.ReturnLoan(
                        ref dataValues,
                        ref sampleInfos);

            return result;
        }

        public override ReturnCode GetKeyValue(
                ref EntityKinematicsInfo key,
                InstanceHandle handle)
        {
            ReturnCode result = base.GetKeyValue(
                        ref key,
                        handle);
            return result;
        }

        public override InstanceHandle LookupInstance(
                EntityKinematicsInfo instance)
        {
            return
                base.LookupInstance(
                        instance);
        }

    }
    #endregion
    
    #region EntityKinematicsInfoDataWriter
    public class EntityKinematicsInfoDataWriter : DDS.OpenSplice.FooDataWriter<EntityKinematicsInfo, __EntityKinematicsInfoMarshaler>, 
                                         IEntityKinematicsInfoDataWriter
    {
        public EntityKinematicsInfoDataWriter(DatabaseMarshaler marshaler)
            : base(marshaler) { }

        public InstanceHandle RegisterInstance(
                EntityKinematicsInfo instanceData)
        {
            return base.RegisterInstance(
                    instanceData,
                    Time.Current);
        }

        public InstanceHandle RegisterInstanceWithTimestamp(
                EntityKinematicsInfo instanceData,
                Time sourceTimestamp)
        {
            return base.RegisterInstance(
                    instanceData,
                    sourceTimestamp);
        }

        public ReturnCode UnregisterInstance(
                EntityKinematicsInfo instanceData,
                InstanceHandle instanceHandle)
        {
            return base.UnregisterInstance(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode UnregisterInstanceWithTimestamp(
                EntityKinematicsInfo instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.UnregisterInstance(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public ReturnCode Write(EntityKinematicsInfo instanceData)
        {
            return base.Write(
                    instanceData,
                    InstanceHandle.Nil,
                    Time.Current);
        }

        public ReturnCode Write(
                EntityKinematicsInfo instanceData,
                InstanceHandle instanceHandle)
        {
            return base.Write(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode WriteWithTimestamp(
                EntityKinematicsInfo instanceData,
                Time sourceTimestamp)
        {
            return base.Write(
                    instanceData,
                    InstanceHandle.Nil,
                    sourceTimestamp);
        }

        public ReturnCode WriteWithTimestamp(
                EntityKinematicsInfo instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.Write(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public ReturnCode Dispose(
                EntityKinematicsInfo instanceData,
                InstanceHandle instanceHandle)
        {
            return base.Dispose(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode DisposeWithTimestamp(
                EntityKinematicsInfo instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.Dispose(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public ReturnCode WriteDispose(
                EntityKinematicsInfo instanceData)
        {
            return base.WriteDispose(
                    instanceData,
                    InstanceHandle.Nil,
                    Time.Current);
        }

        public ReturnCode WriteDispose(
                EntityKinematicsInfo instanceData,
                InstanceHandle instanceHandle)
        {
            return base.WriteDispose(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode WriteDisposeWithTimestamp(
                EntityKinematicsInfo instanceData,
                Time sourceTimestamp)
        {
            return base.WriteDispose(
                    instanceData,
                    InstanceHandle.Nil,
                    sourceTimestamp);
        }

        public ReturnCode WriteDisposeWithTimestamp(
                EntityKinematicsInfo instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.WriteDispose(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public override ReturnCode GetKeyValue(
                ref EntityKinematicsInfo key,
                InstanceHandle instanceHandle)
        {
            return base.GetKeyValue(ref key, instanceHandle);
        }

        public override InstanceHandle LookupInstance(
            EntityKinematicsInfo instanceData)
        {
            return base.LookupInstance(instanceData);
        }
    }
    #endregion

    #region EntityKinematicsInfoTypeSupport
    public class EntityKinematicsInfoTypeSupport : DDS.OpenSplice.TypeSupport
    {
        private static readonly string[] metaDescriptor = {"<MetaData version=\"1.0.0\"><Module name=\"entity\"><Struct name=\"EntityKinematicsInfo\"><Member name=\"id\">",
"<Short/></Member><Member name=\"x\"><Float/></Member><Member name=\"y\"><Float/></Member><Member name=\"z\">",
"<Float/></Member><Member name=\"roll\"><Double/></Member><Member name=\"pitch\"><Double/></Member><Member name=\"yaw\">",
"<Double/></Member></Struct></Module></MetaData>"};

        public EntityKinematicsInfoTypeSupport()
            : base(typeof(EntityKinematicsInfo), metaDescriptor, "entity::EntityKinematicsInfo", "", "id")
        { }


        public override ReturnCode RegisterType(IDomainParticipant participant, string typeName)
        {
            return RegisterType(participant, typeName, new __EntityKinematicsInfoMarshaler());
        }

        public override DDS.OpenSplice.DataWriter CreateDataWriter(DatabaseMarshaler marshaler)
        {
            return new EntityKinematicsInfoDataWriter(marshaler);
        }

        public override DDS.OpenSplice.DataReader CreateDataReader(DatabaseMarshaler marshaler)
        {
            return new EntityKinematicsInfoDataReader(marshaler);
        }
    }
    #endregion

    #region EntityShapeDataReader
    public class EntityShapeDataReader : DDS.OpenSplice.FooDataReader<EntityShape, __EntityShapeMarshaler>, 
                                         IEntityShapeDataReader
    {
        public EntityShapeDataReader(DatabaseMarshaler marshaler)
            : base(marshaler) { }

        public ReturnCode Read(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos)
        {
            return Read(ref dataValues, ref sampleInfos, Length.Unlimited);
        }

        public ReturnCode Read(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples)
        {
            return Read(ref dataValues, ref sampleInfos, maxSamples, SampleStateKind.Any,
                ViewStateKind.Any, InstanceStateKind.Any);
        }

        public ReturnCode Read(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates)
        {
            return Read(ref dataValues, ref sampleInfos, Length.Unlimited, sampleStates,
                viewStates, instanceStates);
        }

        public override ReturnCode Read(
                ref EntityShape[] dataValues,
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
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos)
        {
            return Take(ref dataValues, ref sampleInfos, Length.Unlimited);
        }

        public ReturnCode Take(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples)
        {
            return Take(ref dataValues, ref sampleInfos, maxSamples, SampleStateKind.Any,
                ViewStateKind.Any, InstanceStateKind.Any);
        }

        public ReturnCode Take(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            SampleStateKind sampleStates,
            ViewStateKind viewStates,
            InstanceStateKind instanceStates)
        {
            return Take(ref dataValues, ref sampleInfos, Length.Unlimited, sampleStates,
                viewStates, instanceStates);
        }

        public override ReturnCode Take(
                ref EntityShape[] dataValues,
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
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition)
        {
            return ReadWithCondition(ref dataValues, ref sampleInfos,
                Length.Unlimited, readCondition);
        }

        public override ReturnCode ReadWithCondition(
                ref EntityShape[] dataValues,
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
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            IReadCondition readCondition)
        {
            return TakeWithCondition(ref dataValues, ref sampleInfos,
                Length.Unlimited, readCondition);
        }

        public override ReturnCode TakeWithCondition(
                ref EntityShape[] dataValues,
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
                ref EntityShape dataValue,
                ref SampleInfo sampleInfo)
        {
            ReturnCode result =
                base.ReadNextSample(
                        ref dataValue,
                        ref sampleInfo);
            return result;
        }

        public override ReturnCode TakeNextSample(
                ref EntityShape dataValue,
                ref SampleInfo sampleInfo)
        {
            ReturnCode result =
                base.TakeNextSample(
                        ref dataValue,
                        ref sampleInfo);
            return result;
        }

        public ReturnCode ReadInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return ReadInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode ReadInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return ReadInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode ReadInstance(
                ref EntityShape[] dataValues,
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
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return TakeInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode TakeInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return TakeInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode TakeInstance(
                ref EntityShape[] dataValues,
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
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return ReadNextInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode ReadNextInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return ReadNextInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode ReadNextInstance(
                ref EntityShape[] dataValues,
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
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            InstanceHandle instanceHandle)
        {
            return TakeNextInstance(ref dataValues, ref sampleInfos, Length.Unlimited, instanceHandle);
        }

        public ReturnCode TakeNextInstance(
            ref EntityShape[] dataValues,
            ref SampleInfo[] sampleInfos,
            int maxSamples,
            InstanceHandle instanceHandle)
        {
            return TakeNextInstance(ref dataValues, ref sampleInfos, maxSamples, instanceHandle,
                SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        }

        public override ReturnCode TakeNextInstance(
                ref EntityShape[] dataValues,
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
                ref EntityShape[] dataValues,
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
                ref EntityShape[] dataValues,
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
                ref EntityShape[] dataValues,
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
                ref EntityShape[] dataValues,
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
                ref EntityShape[] dataValues,
                ref SampleInfo[] sampleInfos)
        {
            ReturnCode result =
                base.ReturnLoan(
                        ref dataValues,
                        ref sampleInfos);

            return result;
        }

        public override ReturnCode GetKeyValue(
                ref EntityShape key,
                InstanceHandle handle)
        {
            ReturnCode result = base.GetKeyValue(
                        ref key,
                        handle);
            return result;
        }

        public override InstanceHandle LookupInstance(
                EntityShape instance)
        {
            return
                base.LookupInstance(
                        instance);
        }

    }
    #endregion
    
    #region EntityShapeDataWriter
    public class EntityShapeDataWriter : DDS.OpenSplice.FooDataWriter<EntityShape, __EntityShapeMarshaler>, 
                                         IEntityShapeDataWriter
    {
        public EntityShapeDataWriter(DatabaseMarshaler marshaler)
            : base(marshaler) { }

        public InstanceHandle RegisterInstance(
                EntityShape instanceData)
        {
            return base.RegisterInstance(
                    instanceData,
                    Time.Current);
        }

        public InstanceHandle RegisterInstanceWithTimestamp(
                EntityShape instanceData,
                Time sourceTimestamp)
        {
            return base.RegisterInstance(
                    instanceData,
                    sourceTimestamp);
        }

        public ReturnCode UnregisterInstance(
                EntityShape instanceData,
                InstanceHandle instanceHandle)
        {
            return base.UnregisterInstance(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode UnregisterInstanceWithTimestamp(
                EntityShape instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.UnregisterInstance(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public ReturnCode Write(EntityShape instanceData)
        {
            return base.Write(
                    instanceData,
                    InstanceHandle.Nil,
                    Time.Current);
        }

        public ReturnCode Write(
                EntityShape instanceData,
                InstanceHandle instanceHandle)
        {
            return base.Write(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode WriteWithTimestamp(
                EntityShape instanceData,
                Time sourceTimestamp)
        {
            return base.Write(
                    instanceData,
                    InstanceHandle.Nil,
                    sourceTimestamp);
        }

        public ReturnCode WriteWithTimestamp(
                EntityShape instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.Write(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public ReturnCode Dispose(
                EntityShape instanceData,
                InstanceHandle instanceHandle)
        {
            return base.Dispose(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode DisposeWithTimestamp(
                EntityShape instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.Dispose(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public ReturnCode WriteDispose(
                EntityShape instanceData)
        {
            return base.WriteDispose(
                    instanceData,
                    InstanceHandle.Nil,
                    Time.Current);
        }

        public ReturnCode WriteDispose(
                EntityShape instanceData,
                InstanceHandle instanceHandle)
        {
            return base.WriteDispose(
                    instanceData,
                    instanceHandle,
                    Time.Current);
        }

        public ReturnCode WriteDisposeWithTimestamp(
                EntityShape instanceData,
                Time sourceTimestamp)
        {
            return base.WriteDispose(
                    instanceData,
                    InstanceHandle.Nil,
                    sourceTimestamp);
        }

        public ReturnCode WriteDisposeWithTimestamp(
                EntityShape instanceData,
                InstanceHandle instanceHandle,
                Time sourceTimestamp)
        {
            return base.WriteDispose(
                    instanceData,
                    instanceHandle,
                    sourceTimestamp);
        }

        public override ReturnCode GetKeyValue(
                ref EntityShape key,
                InstanceHandle instanceHandle)
        {
            return base.GetKeyValue(ref key, instanceHandle);
        }

        public override InstanceHandle LookupInstance(
            EntityShape instanceData)
        {
            return base.LookupInstance(instanceData);
        }
    }
    #endregion

    #region EntityShapeTypeSupport
    public class EntityShapeTypeSupport : DDS.OpenSplice.TypeSupport
    {
        private static readonly string[] metaDescriptor = {"<MetaData version=\"1.0.0\"><Module name=\"common\"><Struct name=\"Vector3\"><Member name=\"x\"><Double/>",
"</Member><Member name=\"y\"><Double/></Member><Member name=\"z\"><Double/></Member></Struct><Struct name=\"Triangle\">",
"<Member name=\"p1\"><Type name=\"Vector3\"/></Member><Member name=\"p2\"><Type name=\"Vector3\"/></Member>",
"<Member name=\"p3\"><Type name=\"Vector3\"/></Member></Struct></Module><Module name=\"entity\"><Struct name=\"EntityShape\">",
"<Member name=\"id\"><Short/></Member><Member name=\"triangles_list\"><Sequence><Type name=\"::common::Triangle\"/>",
"</Sequence></Member></Struct></Module></MetaData>"};

        public EntityShapeTypeSupport()
            : base(typeof(EntityShape), metaDescriptor, "entity::EntityShape", "", "id")
        { }


        public override ReturnCode RegisterType(IDomainParticipant participant, string typeName)
        {
            return RegisterType(participant, typeName, new __EntityShapeMarshaler());
        }

        public override DDS.OpenSplice.DataWriter CreateDataWriter(DatabaseMarshaler marshaler)
        {
            return new EntityShapeDataWriter(marshaler);
        }

        public override DDS.OpenSplice.DataReader CreateDataReader(DatabaseMarshaler marshaler)
        {
            return new EntityShapeDataReader(marshaler);
        }
    }
    #endregion

}

