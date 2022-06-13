using System;

using DDS;

namespace DDSHelper
{
    public interface IDDSTopic
    {
        public IDataWriter DataWriter();
        public IDataReader DataReader();
    }

    public class DDSTopic<Topic, Writer, Reader> : System.IDisposable
        where Topic : DDS.OpenSplice.TypeSupport, new()
        where Writer : class, DDS.IDataWriter
        where Reader : class, DDS.IDataReader
        
    {
        private ITopic _topic;
        private Writer _dataWriter;
        private Reader _dataReader;

        public Writer DataWriter { get { return _dataWriter; } }
        public Reader DataReader { get { return _dataReader; } }

        public Writer GetDataWriter() { return _dataWriter; }
        public Reader GetDataReader() { return _dataReader; }

        public DDSTopic(DDSNode node, TopicQos qos, string topicName)
        {
            ReturnCode status;
            Topic topic = new Topic();
            topic.RegisterType(node.Participant, topic.TypeName);

            _topic = node.Participant.CreateTopic(topicName, topic.TypeName, qos);
            ErrorHandler.checkHandle(_topic, "DomainParticipant.CreateTopic");

            // Writer
            DataWriterQos writerQos = null;
            status = node.Publisher.GetDefaultDataWriterQos(ref writerQos);
            ErrorHandler.checkStatus(status, "Publisher.GetDefaultDataWriterQos");
                       

            status = node.Publisher.CopyFromTopicQos(ref writerQos, qos);
            ErrorHandler.checkStatus(status, "Publisher.CopyFromTopicQos");
            _dataWriter = node.Publisher.CreateDataWriter(_topic, writerQos) as Writer;

            //Reader
            DataReaderQos readerQos = null;
            status = node.Subscriber.GetDefaultDataReaderQos(ref readerQos);
            ErrorHandler.checkStatus(status, "Subscriber.GetDefaultDataReaderQos");


            status = node.Subscriber.CopyFromTopicQos(ref readerQos, qos);
            ErrorHandler.checkStatus(status, "Subscriber.CopyFromTopicQos");
            _dataReader = node.Subscriber.CreateDataReader(_topic, readerQos) as Reader;
        }

        public void Dispose()
        {
            throw new System.NotImplementedException();
        }
    }
}
