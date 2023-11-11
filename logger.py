import os
import time
from datetime import date, datetime

class Logger:

    MAX_BACKUP_LINES = 50

    def __init__(self, log_path=os.path.join(".","log.txt")) -> None:
        log_file = open(log_path,"w",encoding="utf-8",buffering=1)
        title_text = f"LAUNCH-OPS@{date.today()}-{datetime.now().strftime('%H-%M')}\n"
        log_file.write(title_text)

        self.log_path = log_path
        self.log_file = log_file
        self.start_time = time.time()
        self.title_text = title_text
        self.backup_record = ""
        self.backup_record_count = 0

    def get_logger(self):
        return self.log_file
    
    def get_log_time(self):
        return time.time()-self.start_time
    
    def log(self, text):
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
        original_log_path = self.log_path
        recover_log_path = original_log_path[:-4]+"_Rec.txt"
        log_file = open(recover_log_path,"w",encoding="utf-8",buffering=1)

        recover_text = self.title_text+self.backup_record
        recover_text += f"RECOVER-TIME@{datetime.now().strftime('%H-%M-%S')}\n"
        recover_text += f"CAUSE-{err}\n"
        log_file.writelines(recover_text)

        self.log_file = log_file
        self.log_path = recover_log_path
        self.log(stashed_text)
    
    def __del__(self):
        self.log("COMPLETED-OPS")
        self.log_file.close()
