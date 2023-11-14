import os
import time
from datetime import date, datetime

class Logger:

    MAX_BACKUP_LINES = 50

    def __init__(self, log_path=os.path.join(".","log.txt")) -> None:
        """initialize an universal data logger

        Args:
            log_path (str, optional): path of the log file. Defaults to os.path.join(".","log.txt").
        """
        # open a log text file with 1 line buffer
        log_file = open(log_path,"w",encoding="utf-8",buffering=1)
        title_text = f"LAUNCH-OPS@{date.today()}-{datetime.now().strftime('%H-%M')}\n"
        log_file.write(title_text)

        # initialize class instance fields
        self.log_path = log_path
        self.log_file = log_file
        # define system clock to be aligned with logger clock
        self.start_time = time.time()
        self.title_text = title_text
        self.backup_record = ""
        self.backup_record_count = 0
    
    def get_log_time(self):
        """getter function for universal mission time

        Returns:
            float: elapsed mission time
        """
        return time.time()-self.start_time
    
    def log(self, text):
        """format input text and log to the data file

        Args:
            text (str): raw text that needs to be logged
        """
        # format time stamped log
        time_stamp = time.time()-self.start_time
        formated_text = f"{time_stamp:0.3f}:{text}\n"
        try:
            self.log_file.write(formated_text)
        except Exception as err:
            self.__recover(text,err)

        # update backup record
        if self.backup_record_count == Logger.MAX_BACKUP_LINES:
            self.backup_record = ""
            self.backup_record_count = 0
        self.backup_record += formated_text
        self.backup_record_count += 1

    def __recover(self, stashed_text, err):
        """re-create the log Text IO and attempt to restore text

        Args:
            stashed_text (str): text that was attempted to be logged
            err (Exception): exception encountered during log() call
        """
        # re-open a log recovery text file
        original_log_path = self.log_path
        recover_log_path = original_log_path[:-4]+"_Rec.txt"
        log_file = open(recover_log_path,"w",encoding="utf-8",buffering=1)

        # re-log back up record and indication of recovery time
        recover_text = self.title_text+self.backup_record
        recover_text += f"RECOVER-TIME@{datetime.now().strftime('%H-%M-%S')}\n"
        recover_text += f"CAUSE-{err}\n"
        log_file.writelines(recover_text)

        self.log_file = log_file
        self.log_path = recover_log_path
        self.log(stashed_text)
    
    def clean(self):
        """close the file
        """
        self.log("COMPLETED-OPS")
        self.log_file.close()
