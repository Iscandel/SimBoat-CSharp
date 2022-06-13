using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using DDS;
using DDSHelper;

using entity;

public class PathResolver
{
    public static Dictionary<string, string> _resourcePath;
    public static List<string> _directories;

    static PathResolver()
    {
        _resourcePath = new Dictionary<string, string>();
        _directories = new List<string>();

        _directories.Add("Prefabs/");
        _directories.Add("Resources/");

        foreach(string s in _directories)
            foreach(GameObject go in Resources.LoadAll(s))
                _resourcePath[go.name.ToLower()] = s;
    }

    public static void AddSearchPath(string path)
    {
        _directories.Add(path);
    }

    public static bool RemoveSearchPath(string path)
    {
        return  _directories.Remove(path);
    }

    public static string Resolve(string prefabName)
    {
        _resourcePath.TryGetValue(prefabName.ToLower(), out string path);
        return path;
    }

}
public class DDSEntitySpawner : MonoBehaviour
{
    EntityInfoDataReader _reader;

    private Dictionary<int, GameObject> _entityById;

    // Start is called before the first frame update
    void Start()
    {
        _entityById = new Dictionary<int, GameObject>();


        //System.Type type = typeof(DDSTopic<EntityInfoTypeSupport, EntityInfoDataWriter, EntityInfoDataReader>);
        ////if (DDSManager.HasReader(type))
        //if(!DDSManager.Instance.HasTopic(type))
        //{ 
        //    DDSManager.Instance.AddTopic(type, "entityTopic");          
        //}

        //_reader = DDSManager.Instance.GetReader<EntityInfoTypeSupport, EntityInfoDataWriter, EntityInfoDataReader>();

        (_, _reader) = CreateTopicIfNonExistent<EntityInfoTypeSupport, EntityInfoDataWriter, EntityInfoDataReader>("entity_EntityInfo");// entityTopic");
    }

    // Update is called once per frame
    void Update()
    {
        EntityInfo[] msgEntity = null;
        DDS.SampleInfo[] infoSeq = null;
        ReturnCode status;

        status = _reader.Take(ref msgEntity, ref infoSeq, Length.Unlimited, SampleStateKind.Any, ViewStateKind.New, InstanceStateKind.Alive);
        ErrorHandler.checkStatus(status, "DataReader.Take");
        for (int i = 0; i < msgEntity.Length; i++)
        {
            if (infoSeq[i].ValidData)
            {
                Debug.Log("=== [Subscriber] message received :");
                Debug.Log("    id  : " + msgEntity[i].id);
                Debug.Log("    type : \"" + msgEntity[i].type + "\"");

                if(!HasId(msgEntity[i].id))
                {
                    string path = PathResolver.Resolve(msgEntity[i].type);
                    GameObject prefab = Resources.Load<GameObject>(path + msgEntity[i].type);
                    GameObject instance = Instantiate(prefab, Vector3.zero, Quaternion.identity);

                    if (instance)
                    {
                        AddEntity(msgEntity[i].id, instance);
                        Debug.Log("Created entity (id " + msgEntity[i].id + "): " + msgEntity[i].type);
                    }
                    else 
                    {
                        Debug.LogWarning("Failed to create entity (id " + msgEntity[i].id + "): " + msgEntity[i].type);
                    }
                }
            }
        }
        status = _reader.ReturnLoan(ref msgEntity, ref infoSeq);
        ErrorHandler.checkStatus(status, "DataReader.ReturnLoan");
    }

    public bool HasId(int id)
    {
        return _entityById.ContainsKey(id);
    }

    public void AddEntity(int id, GameObject instance)
    {
        _entityById.Add(id, instance);

        EntityStatusInfo info = instance.AddComponent<EntityStatusInfo>();
        info.Id = id;
        var entityKinematics = instance.AddComponent<DDSEntityKinematics>();
        entityKinematics.enabled = true;
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

        Reader reader = DDSManager.Instance.GetReader<EntityInfoDataReader>() as Reader;
        Writer writer = DDSManager.Instance.GetWriter<EntityInfoDataWriter>() as Writer;

        return (writer: writer, reader: reader);
    }
}

