using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading;

using DDS;
using DDSHelper;
using chat;

//public class DDSPublisher : MonoBehaviour
//{
//    private DDSNode _node;
//    private DDSTopic<ChatMsgTypeSupport, ChatMsgDataWriter, ChatMsgDataReader> _topic;
//    private chat.ChatMsg _msg;

//    // Start is called before the first frame update
//    void Start()
//    {
//        _node = new DDSNode("");// "test");
//        TopicQos qos = null;
//        ReturnCode status = _node.Participant.GetDefaultTopicQos(ref qos);
//        ErrorHandler.checkStatus(status, "DomainParticipant.GetDefaultTopicQos");
//        qos.Durability.Kind = DDS.DurabilityQosPolicyKind.TransientDurabilityQos;
//        qos.Reliability.Kind = DDS.ReliabilityQosPolicyKind.BestEffortReliabilityQos;
//        qos.History.Kind = DDS.HistoryQosPolicyKind.KeepLastHistoryQos;
//        qos.History.Depth = 5;

//        _topic =
//            new DDSTopic<ChatMsgTypeSupport, ChatMsgDataWriter, ChatMsgDataReader>(_node, qos, "chatTopic");

//        _msg = new ChatMsg();
//        _msg.id = 10;
//        _msg.cpt = 0;

//        _topic.DataWriter.RegisterInstance(_msg);
//    }

//    // Update is called once per frame
//    void Update()
//    {
//        if (_msg != null)
//        {
//            _msg.cpt++;
//            _msg.text = "Hello ! " + _msg.cpt;

//            _topic.DataWriter.Write(_msg);
//            Debug.Log(_msg.text);

//            Thread.Sleep(1000);
//        }
//    }
//}

using entity;

public class DDSPublisher : MonoBehaviour
{
    private EntityInfoDataWriter _writer;
    private entity.EntityInfo _msg;

    // Start is called before the first frame update
    void Start()
    {
        System.Type type = typeof(DDSTopic<EntityInfoTypeSupport, EntityInfoDataWriter, EntityInfoDataReader>);
        //if (DDSManager.HasReader(type))
        if (!DDSManager.Instance.HasTopic(type))
        {
            DDSManager.Instance.AddTopic(type, "entityTopic");
        }

        _writer = DDSManager.Instance.GetWriter<EntityInfoTypeSupport, EntityInfoDataWriter, EntityInfoDataReader>();
        _msg = new EntityInfo();
        _msg.id = 1;
        _msg.type = "SmallBoat";

        _writer.RegisterInstance(_msg);
        _writer.Write(_msg);
    }

    // Update is called once per frame
    void Update()
    {
    }
}
