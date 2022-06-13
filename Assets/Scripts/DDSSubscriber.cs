using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using DDS;
using DDSHelper;
using chat;
using System;

public class DDSSubscriber : MonoBehaviour
{
    private DDSNode _node;
    private DDSTopic<ChatMsgTypeSupport, ChatMsgDataWriter, ChatMsgDataReader> _topic;
    private chat.ChatMsg _msg;


        // Start is called before the first frame update
    void Start()
    {
        _node = new DDSNode("");// "test");
        TopicQos qos = null;
        ReturnCode status = _node.Participant.GetDefaultTopicQos(ref qos);
        ErrorHandler.checkStatus(status, "DomainParticipant.GetDefaultTopicQos");
        qos.Durability.Kind = DDS.DurabilityQosPolicyKind.TransientDurabilityQos;
        qos.Reliability.Kind = DDS.ReliabilityQosPolicyKind.ReliableReliabilityQos;
        qos.History.Kind = DDS.HistoryQosPolicyKind.KeepLastHistoryQos;
        qos.History.Depth = 5;

        _topic =
            new DDSTopic<ChatMsgTypeSupport, ChatMsgDataWriter, ChatMsgDataReader>(_node, qos, "chatTopic");

        Debug.Log("=== [Subscriber] Ready ...");
    }

    // Update is called once per frame
    void Update()
    {
        ChatMsg[] msgSeq = null;
        DDS.SampleInfo[] infoSeq = null;
        ReturnCode status;

        ChatMsgDataReader reader = _topic.DataReader;

        //ChatMsg msg = new ChatMsg();
        //msg.id = 10;
        //InstanceHandle handle = reader.LookupInstance(msg);
        //if (!handle.Equals(InstanceHandle.Nil))
        //{
        //    status = reader.TakeInstance(ref msgSeq, ref infoSeq, handle);
        //    ErrorHandler.checkStatus(status, "DataReader.Take");
        //    for (int i = 0; i < msgSeq.Length; i++)
        //    {
        //        if (infoSeq[i].ValidData)
        //        {
        //            Console.WriteLine("=== [Subscriber] message received :");
        //            Console.WriteLine("    Text  : {0}", msgSeq[i].text);
        //            Console.WriteLine("    cpt : \"" + msgSeq[i].cpt + "\"");
        //        }
        //    }

        //    status = reader.ReturnLoan(ref msgSeq, ref infoSeq);
        //    ErrorHandler.checkStatus(status, "DataReader.ReturnLoan");
        //}


        status = reader.Take(ref msgSeq, ref infoSeq, Length.Unlimited, SampleStateKind.Any, ViewStateKind.Any, InstanceStateKind.Any);
        ErrorHandler.checkStatus(status, "DataReader.Take");
        for (int i = 0; i < msgSeq.Length; i++)
        {
            if (infoSeq[i].ValidData)
            {
                Debug.Log("=== [Subscriber] message received :");
                Debug.Log("    Text  : " +  msgSeq[i].text);
                Debug.Log("    cpt : \"" + msgSeq[i].cpt + "\"");
            }
        }
        status = reader.ReturnLoan(ref msgSeq, ref infoSeq);
        ErrorHandler.checkStatus(status, "DataReader.ReturnLoan");
    }
}
