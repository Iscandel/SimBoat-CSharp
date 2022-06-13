using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using DDS;
using DDSHelper;

public class DDSManager : MonoBehaviour
{
    private static DDSManager _instance;
    private List<object> _topicList;
    private DDSNode _node;
    public static DDSManager Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindObjectOfType<DDSManager>();
            }

            if (_instance == null)
            {
                _instance = new DDSManager();
            }

            return _instance;
        }
    }

    private DDSManager()
    {
        _topicList = new List<object>();
        _node = new DDSNode("");
    }

    public bool AddTopic(System.Type type, string topicName)
    {
        foreach (object @object in _topicList)
        {
            if (@object.GetType() == type)
                return false;
        }

        TopicQos qos = null;
        ReturnCode status = _node.Participant.GetDefaultTopicQos(ref qos);
        ErrorHandler.checkStatus(status, "DomainParticipant.GetDefaultTopicQos");
        qos.Durability.Kind = DDS.DurabilityQosPolicyKind.TransientDurabilityQos;
        qos.Reliability.Kind = DDS.ReliabilityQosPolicyKind.ReliableReliabilityQos;//.BestEffortReliabilityQos;
        qos.History.Kind = DDS.HistoryQosPolicyKind.KeepLastHistoryQos;
        qos.History.Depth = 1;

        _topicList.Add(System.Activator.CreateInstance(type, new object[]{ _node, qos, topicName}));
        return true;
    }

    public bool AddTopic<Topic, Writer, Reader >(string topicName)
        where Topic : DDS.OpenSplice.TypeSupport, new()
        where Writer : class, DDS.IDataWriter
        where Reader : class, DDS.IDataReader
    {
        foreach (object @object in _topicList)
        {
            if (@object.GetType() == typeof(Topic))
                return false;
        }

        TopicQos qos = null;
        ReturnCode status = _node.Participant.GetDefaultTopicQos(ref qos);
        ErrorHandler.checkStatus(status, "DomainParticipant.GetDefaultTopicQos");
        qos.Durability.Kind = DDS.DurabilityQosPolicyKind.TransientDurabilityQos;
        qos.Reliability.Kind = DDS.ReliabilityQosPolicyKind.ReliableReliabilityQos;//.BestEffortReliabilityQos;
        qos.History.Kind = DDS.HistoryQosPolicyKind.KeepLastHistoryQos;
        qos.History.Depth = 5;

        _topicList.Add(new DDSTopic<Topic, Writer, Reader>(_node, qos, topicName));
        return true;
    }

    public bool HasTopic(System.Type type)
    {
        foreach (object @object in _topicList)
        {
            if (@object.GetType() == type)
                return true;
        }

        return false;
    }

    public bool HasTopic<Type>()
    {
        foreach (object @object in _topicList)
        {
            if ((Type)@object != null)
                return true;
        }

        return false;
    }

    //    public bool HasReader(System.Type type)
    //{
    //    foreach(object @object in _topicList)
    //    {
    //        if (@object.GetType() == type)
    //    }
    //}

    public Reader GetReader<Topic, Writer, Reader>()
        where Topic : DDS.OpenSplice.TypeSupport, new()
        where Writer : class, DDS.IDataWriter
        where Reader : class, DDS.IDataReader
    {
        foreach (object @object in _topicList)
        {
            DDSTopic<Topic, Writer, Reader> topic = @object as DDSTopic<Topic, Writer, Reader>;
            if (topic != null)
            {
                //delegate void D3();
                //object del = System.Delegate.CreateDelegate(typeof(Topic), topic, "Datareader");
                object result = topic.GetType().GetMethod("GetDataReader").Invoke(topic, null);
                if(result != null && (result as Reader) != null)
                    return result as Reader;
            }
        }

        return default(Reader);
    }

    public Reader GetReader<Reader>()
        where Reader : class, DDS.IDataReader
    {
        foreach (object @object in _topicList)
        {
            //delegate void D3();
            //object del = System.Delegate.CreateDelegate(typeof(Topic), topic, "Datareader");
            object result = @object.GetType().GetMethod("GetDataReader").Invoke(@object, null);
            if( result != null && (result as Reader) != null)
                return result as Reader;
        }

        return default(Reader);
    }

    public Writer GetWriter<Topic, Writer, Reader>()
        where Topic : DDS.OpenSplice.TypeSupport, new()
        where Writer : class, DDS.IDataWriter
        where Reader : class, DDS.IDataReader
    {
        foreach (object @object in _topicList)
        {
            DDSTopic<Topic, Writer, Reader> topic = @object as DDSTopic<Topic, Writer, Reader>;
            if (topic != null)
            {
                //delegate void D3();
                //object del = System.Delegate.CreateDelegate(typeof(Topic), topic, "Datareader");
                object result = topic.GetType().GetMethod("GetDataWriter").Invoke(topic, null);
                if(result != null && (result as Writer) != null)
                    return result as Writer;
            }
        }

        return default(Writer);
    }

    public Writer GetWriter<Writer>()
        where Writer : class, DDS.IDataWriter
    {
        foreach (object @object in _topicList)
        {
            //delegate void D3();
            //object del = System.Delegate.CreateDelegate(typeof(Topic), topic, "Datareader");
            object result = @object.GetType().GetMethod("GetDataWriter").Invoke(@object, null);
            if(result != null && (result as Writer) != null)
                return result as Writer;
        }

        return default(Writer);
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
