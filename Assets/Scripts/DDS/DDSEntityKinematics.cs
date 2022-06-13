using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using DDS;
using DDSHelper;
using entity;

public class DDSEntityKinematics : MonoBehaviour
{
    EntityKinematicsInfoDataReader _reader;
    InstanceHandle _kinematicsHandle;

    // Start is called before the first frame update
    void Start()
    {
        (_, _reader) = CreateTopicIfNonExistent<EntityKinematicsInfoTypeSupport, 
                    EntityKinematicsInfoDataWriter, EntityKinematicsInfoDataReader>("entity_EntityKinematicsInfo");
        Debug.Log("Created reader kinematics");
    }

    // Update is called once per frame
    void Update()
    {
        
        DDS.SampleInfo[] infoSeq = null;
        ReturnCode status;

        if(_kinematicsHandle == InstanceHandle.Nil)
        {
            EntityStatusInfo entityInfo = GetComponent<EntityStatusInfo>();
            var msgEntityKinematics = new EntityKinematicsInfo();
            msgEntityKinematics.id = (short)entityInfo.Id;
            Debug.Log("Got kinematics handle");
            Debug.Log(msgEntityKinematics.id);
            Debug.Log(_reader);
            _kinematicsHandle = _reader.LookupInstance(msgEntityKinematics);
        }
        
        if (_kinematicsHandle != InstanceHandle.Nil)
        {
            Debug.Log("Enter update");
            EntityKinematicsInfo[] msgEntityKinematics = null;
            status = _reader.TakeInstance(ref msgEntityKinematics, ref infoSeq, 1, _kinematicsHandle);
            ErrorHandler.checkStatus(status, "DataReader.Take");
            if(status == ReturnCode.Ok)
            { 
                if (infoSeq[0].ValidData)
                {
                    //Debug.Log("=== [Kinematics] message received :");

                    transform.position = new Vector3(msgEntityKinematics[0].y, -msgEntityKinematics[0].z, msgEntityKinematics[0].x);
                    transform.rotation = Quaternion.Euler(
                        (float)-msgEntityKinematics[0].pitch,
                        (float)msgEntityKinematics[0].yaw,
                        (float)-msgEntityKinematics[0].roll
                    );
                }
            }

            status = _reader.ReturnLoan(ref msgEntityKinematics, ref infoSeq);
            ErrorHandler.checkStatus(status, "DataReader.ReturnLoan");
            Debug.Log("End update");
        }
    }

    public (Writer writer, Reader reader) CreateTopicIfNonExistent<Topic, Writer, Reader>(string topicName)
        where Topic : DDS.OpenSplice.TypeSupport, new()
        where Writer : class, DDS.IDataWriter
        where Reader : class, DDS.IDataReader
    {
        System.Type type = typeof(DDSTopic<Topic, Writer, Reader>);
        //if (DDSManager.HasReader(type))
        if (!DDSManager.Instance.HasTopic(type))
        {
            DDSManager.Instance.AddTopic(type, topicName);
        }

        Reader reader = DDSManager.Instance.GetReader<Reader>();// as Reader;
        Writer writer = DDSManager.Instance.GetWriter<Writer>();// as Writer;

        return (writer: writer, reader: reader);
    }
}
