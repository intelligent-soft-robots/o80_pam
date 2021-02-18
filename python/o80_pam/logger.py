import os
from multiprocessing import Process
from datetime import datetime
import o80
import o80_pam
import shared_memory
import copy


def _set_start(segment_id,logger_id):
    shared_memory.set_bool(segment_id,logger_id,True)

def _set_stop(segment_id,logger_id):
    shared_memory.set_bool(segment_id,logger_id,False)

def _should_stop(segment_id,logger_id):
    return not shared_memory.get_bool(segment_id,logger_id)


# runs a loop reading observations, serializing them,
# and writting them in the file
def _log(segment_id,file_path,frequency,logger_id):
    # creating an o80 frontend
    try :
        frontend = o80_pam.FrontEnd(segment_id)
    except:
        print(("\nfailed to start an o80 frontend on segment_id: {}. "
               "Is a corresponding robot running ?\n").format(segment_id))
        return
    # serializer will compress observation instances into string
    serializer = o80_pam.Serializer()
    # setting the collecting loop frequency
    frequency_manager = o80.FrequencyManager(frequency)
    latest = None
    # running the loop
    with open(file_path,"wb+") as f:
        # any other process may stop this loop by calling
        # _set_stop
        while not _should_stop(segment_id,logger_id):
            if latest is None:
                # first call
                observations = [frontend.latest()]
            else:
                # reading all new observations written by the backend
                # since the last pass of this loop
                observations = frontend.get_observations_since(latest+1)
            for observation in observations:
                # serializing and writting all observations
                f.write(serializer.serialize(observation))
                f.flush()
            if observations:
                # keeping track of the latest observation written
                latest = observations[-1].get_iteration()
            # running at desired frequency
            frequency_manager.wait()

    
class Logger:

    """
    Class that will spawn a process that will read the observations
    generated by an o80 backend, and writing them into a binary file.
    Files written by this class can be read by the read_file function of
    this package. Note that this class can be used as context manager.
    
    :param str segment_id: segment_id of the backend
    :param str file_path: absolute path to the file to be written. The file
    will be overwritten if it already exists
    :param float frequency: collecting frequency. The collecting process will
    collect all new observations generated between two of its iteration, so this
    parameter may not be as critical as it seems.
    """
    
    def __init__(self,segment_id,file_path,frequency=500.):

        # throwing exception if the folder of file_path
        # does not exists or is not writable
        filename = os.path.basename(file_path)
        folder = file_path[:-len(filename)]
        if not os.path.isdir(folder):
            raise FileNotFoundError("Failed to find directory: {}".format(folder))
        if not os.access(folder,os.W_OK):
            raise FileNotFoundError("The directory {} does not seem to be writable".format(folder))
        self._file_path = file_path
        self._segment_id = segment_id
        self._frequency = frequency
        self._id = str(id(self))
        
    def start(self):
        """
        starts the observations collecting process
        """
        try:
            frontend = o80_pam.FrontEnd(self._segment_id)
            del frontend
        except Exception as e:
            raise Exception("Failed to create an o80 frontend"+
                            "on segment_id {}: {}".format(self._segment_id,e))
            
        self._process = Process(target=_log,args=(self._segment_id,
                                                  self._file_path,
                                                  self._frequency,
                                                  self._id))
        _set_start(self._segment_id,self._id)
        self._process.start()

    def stop(self):
        """
        stops the observations collecting process
        """
        if hasattr(self,"_process"):
            _set_stop(self._segment_id,self._id)
            self._process.join()
    
    def __enter__(self):
        """
        For usage of this class as a context manager
        """
        self.start()
        return self

    def __exit__(self,_,__,___): 
        """
        For usage of this class as a context manager
        """
        self.stop()


def read_file(file_path):
    """
    returns a generator of the observations stored in a file
    created by an instance of Logger.
    :param str file_path: path to the file to read
    :raises :py:class:`FileNotFoundError`: if the the file does not exists
    :returns: generator of instances of Observation 
    """
    if not os.path.isfile(file_path):
        raise FileNotFoundError("failed to find {}".format(file_path))
    # a serializer will take a string as input and generate a corresponding instance
    # of observation
    serializer = o80_pam.Serializer()
    # size of a string representing an observation 
    serialized_size = o80_pam.Serializer.serializable_size()
    with open(file_path,"rb") as f:
        # reading one serialized instance of observation 
        instance_str = f.read(serialized_size)
        while instance_str:
            # deserializing it into an observation instance
            observation = serializer.deserialize(instance_str)
            # returing the instance
            yield observation
            instance_str = f.read(serialized_size)
    # end of file
    return


class _File:

    # convenience class for dealing with files of path
    # /tmp/o80_pam_observations_num_date
    # where num is a counter (or file id)
    # and date is the date and time of the file creation
    
    def __init__(self,path,num,date):
        self.path = path # absolute path to the file (name included)
        self.num = num # num counter
        self.date = date # date

    @staticmethod
    def create(prefix,path):
        # given an path (i.e. /tmp/o80_pam_observations_num_date)
        # and a prefix (o80_pam_observations_)
        # creates a corresponding instance of _File
        # (i.e. extracts num and date from the path)
        filename = os.path.basename(path)
        index = filename.rfind("_")
        num = int(filename[len(prefix):index])
        date = filename[index:]
        return _File(path,num,index)
    

class FileManager:

    """
    Convenience class for ordering the files generated
    by an instance of Logger. It will propose names for new
    files, by looking at the counter of existing file and the current date.
    e.g. if the file /tmp/o80_pam_observations_5_date already exists,
    it will propose /tmp/o80_pam_observations_6_date as a suitable
    new file path to write data into.
    """
    
    def __init__(self,
                 root_folder="/tmp/",
                 prefix="o80_pam_observations_"):
        self._root_folder = root_folder
        self._prefix = prefix

    def list_files(self):
        """
        Returns the list of absolute path of existing files
        """
        files = [f for f in os.listdir(self._root_folder)
                 if os.path.isfile(os.path.join(self._root_folder,f))]
        files = [os.path.join(self._root_folder,f) for f in files
                 if f.startswith(self._prefix)]
        return [_File.create(self._prefix,f) for f in files]

            
    def next(self):
        """
        Propose a path for a new file. The proposed filename is
        informative, i.e. it provides a new num value and the current date
        and time. e.g. if the file /tmp/o80_pam_observations_5_date already exists,
        it will propose /tmp/o80_pam_observations_6_date
        """
        now = datetime.now()
        date_time = now.strftime("%m-%d-%Y.%H:%M:%S")
        files = self.list_files()
        if not files:
            return os.path.join(self._root_folder,self._prefix+"1_"+date_time)
        newest_files = sorted(files,key=lambda f: f.num)[-1]
        return os.path.join(self._root_folder,self._prefix+str(newest_files.num+1)+"_"+date_time)

    def latest(self):
        """
        Returns the path to the file with the higher num, which is likely to be the most
        recent generated file
        """
        files = self.list_files()
        newest_files = sorted(files,key=lambda f: f.num)[-1]
        return newest_files.path
    


    
        
        
