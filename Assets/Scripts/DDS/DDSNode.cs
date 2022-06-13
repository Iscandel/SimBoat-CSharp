using System;

using DDS;

namespace DDSHelper
{
    public class DDSNode : System.IDisposable
    {
        private DomainParticipantFactory _factory;
        private IDomainParticipant _participant;
        private ISubscriber _subscriber;
        private string _partitionName;

        public IDomainParticipant? Participant { get { return _participant; } }
        public IPublisher? Publisher { get; }
        public ISubscriber? Subscriber => _subscriber;

        public DDSNode(string partitionName)
        {
            _partitionName = partitionName;

            _factory = DDS.DomainParticipantFactory.Instance;
            ErrorHandler.checkHandle(_factory, "DDS.DomainParticipantFactory.Instance");

            DDS.DomainParticipantQos participantQos = new DomainParticipantQos();
        _factory.GetDefaultParticipantQos(ref participantQos);

            _participant = _factory.CreateParticipant(DomainId.Default);//, participantQos);
            ErrorHandler.checkHandle(_participant, "DDS.DomainParticipantFactory.Instance");
            Publisher = createPublisher(_participant, _partitionName);
        _subscriber = createSubscriber(_participant, _partitionName);
        }

        private IPublisher createPublisher(IDomainParticipant participant, string partitionName)
        {
            PublisherQos publisherQos = new PublisherQos();
            ReturnCode status = _participant.GetDefaultPublisherQos(ref publisherQos);
            ErrorHandler.checkStatus(status, "DomainParticipant.GetDefaultPublisherQos");

            publisherQos.Partition.Name = new string[1];
            publisherQos.Partition.Name[0] = partitionName;
            IPublisher publisher = _participant.CreatePublisher(publisherQos);
            ErrorHandler.checkHandle(publisher, "DomainParticipant.CreatePublisher");

            return publisher;
        }

        private ISubscriber createSubscriber(IDomainParticipant participant, string partitionName)
        {
            SubscriberQos subscriberQoS = new SubscriberQos();
            ReturnCode status = _participant.GetDefaultSubscriberQos(ref subscriberQoS);
            ErrorHandler.checkStatus(status, "DomainParticipant.GetDefaultSubscriberQos");

            subscriberQoS.Partition.Name = new string[1];
            subscriberQoS.Partition.Name[0] = partitionName;
            ISubscriber subscriber = _participant.CreateSubscriber(subscriberQoS);
            ErrorHandler.checkHandle(subscriber, "DomainParticipant.CreateSubscriber");

            return subscriber;
        }

        public void Dispose()
        {
            ReturnCode status;
            status = _participant.DeletePublisher(Publisher);
            ErrorHandler.checkStatus(status, "DomainParticipant.DeletePublisher");
            status = _participant.DeleteSubscriber(_subscriber);
            ErrorHandler.checkStatus(status, "DomainParticipant.DeleteSubscriber");
            status = _factory.DeleteParticipant(_participant);
            ErrorHandler.checkStatus(status, "DomainParticipantFactory.DeleteParticipant");
        }
    }
}
