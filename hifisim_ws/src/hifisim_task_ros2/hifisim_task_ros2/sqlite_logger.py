#
# sqlite_logger.py
# Coordinate system used in this class is NWU
# 
import os
import sqlite3
from datetime import datetime
from geometry_msgs.msg import Point, Quaternion

class SQLiteLogger:
    def __init__(self, base_dir='~/hifisim_db'):
        self.base_dir = os.path.expanduser(base_dir)
        
        # Pose database references
        self.pose_db_connection = None
        self.pose_db_cursor = None
        self.pose_db_path = None
        
        # Collision database references
        self.collision_db_connection = None
        self.collision_db_cursor = None
        self.collision_db_path = None

        self.log_dir = None

    def setup_logging(self):
        # Create base directory if it doesn't exist
        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)
            # print(f"[sqlite] Created directory: {self.base_dir}")

        # Create a timestamped subdirectory
        now = datetime.now()
        date_time = now.strftime("%Y-%m-%d-%H%M%S")
        self.log_dir = os.path.join(self.base_dir, f'performance-{date_time}')
        os.makedirs(self.log_dir)
        # print(f"[sqlite] Created directory: {self.log_dir}")

        # Create SQLite database for NWU poses
        self.pose_db_path = os.path.join(self.log_dir, 'nwu_pose.db')
        self.pose_db_connection = sqlite3.connect(self.pose_db_path)
        self.pose_db_cursor = self.pose_db_connection.cursor()

        self.pose_db_cursor.execute('''
            CREATE TABLE IF NOT EXISTS nwu_pose (
                agent_id INTEGER,
                header_stamp_sec INTEGER,
                header_stamp_nanosec INTEGER,
                header_frame_id TEXT,
                position_x REAL,
                position_y REAL,
                position_z REAL,
                orientation_x REAL,
                orientation_y REAL,
                orientation_z REAL,
                orientation_w REAL,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        self.pose_db_connection.commit()

        # print(f"[sqlite] database for NWU poses initialized at {self.pose_db_path}", flush=True)

        # Create SQLite database for collision/crash data
        self.collision_db_path = os.path.join(self.log_dir, 'collision_crash.db')
        self.collision_db_connection = sqlite3.connect(self.collision_db_path)
        self.collision_db_cursor = self.collision_db_connection.cursor()

        self.collision_db_cursor.execute('''
            CREATE TABLE IF NOT EXISTS collision_crash (
                agent_id INTEGER,
                stamp_sec INTEGER,
                stamp_nsec INTEGER,
                is_collision BOOLEAN,
                is_crashed BOOLEAN,
                position_x REAL,
                position_y REAL,
                position_z REAL,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        self.collision_db_connection.commit()

        # print(f"[sqlite] database for Collision/Crash initialized at {self.collision_db_path}", flush=True)

    def log_pose(self, agent_id, stamp_sec, stamp_nanosec, frame_id, position, orientation):
        if not self.pose_db_connection:
            raise Exception("NWU Pose database connection is not initialized. Call setup_logging() first.")
        
        self.pose_db_cursor.execute('''
            INSERT INTO nwu_pose (
                agent_id, header_stamp_sec, header_stamp_nanosec,
                header_frame_id, position_x, position_y, position_z,
                orientation_x, orientation_y, orientation_z, orientation_w
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            agent_id, stamp_sec, stamp_nanosec, frame_id,
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w
        ))
        self.pose_db_connection.commit()

    def log_collision_crash(self, agent_id, stamp_sec, stamp_nsec, is_collision, is_crashed, position):
        if not self.collision_db_connection:
            raise Exception("Collision/Crash database connection is not initialized. Call setup_logging() first.")
        
        self.collision_db_cursor.execute('''
            INSERT INTO collision_crash (
                agent_id, stamp_sec, stamp_nsec,
                is_collision, is_crashed,
                position_x, position_y, position_z
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            agent_id,
            stamp_sec,
            stamp_nsec,
            is_collision,
            is_crashed,
            position.x,
            position.y,
            position.z
        ))
        self.collision_db_connection.commit()

    def retrieve_agent_poses(self, agent_id):
        """
        Retrieve pose data for the given agent_id from the NWU pose database.
        
        Returns a list of dictionaries. Each dictionary has:
          - 'pose_time': floating-point time in seconds (header_stamp_sec + header_stamp_nanosec/1e9)
          - 'position': a Point object with attributes x, y, z
          - 'orientation': a Quaternion object with attributes x, y, z, w
        """
        query = '''
            SELECT header_stamp_sec, header_stamp_nanosec,
                   position_x, position_y, position_z,
                   orientation_x, orientation_y, orientation_z, orientation_w
            FROM nwu_pose
            WHERE agent_id = ?
            ORDER BY header_stamp_sec ASC, header_stamp_nanosec ASC
        '''
        self.pose_db_cursor.execute(query, (agent_id,))
        rows = self.pose_db_cursor.fetchall()

        agent_poses = []
        for row in rows:
            stamp_sec, stamp_nanosec, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w = row
            pose_time = stamp_sec + stamp_nanosec / 1e9
            position = Point(x=float(pos_x), y=float(pos_y), z=float(pos_z))
            orientation = Quaternion(
                x=float(ori_x) if ori_x is not None else 0.0,
                y=float(ori_y) if ori_y is not None else 0.0,
                z=float(ori_z) if ori_z is not None else 0.0,
                w=float(ori_w) if ori_w is not None else 1.0
            )
            agent_poses.append({
                'pose_time': pose_time,
                'position': position,
                'orientation': orientation
            })

        return agent_poses

    def close(self):
        if self.pose_db_connection:
            self.pose_db_connection.close()
            print(f"[sqlite] Closed NWU Pose SQLite database at {self.pose_db_path}")

        if self.collision_db_connection:
            self.collision_db_connection.close()
            print(f"[sqlite] Closed Collision/Crash SQLite database at {self.collision_db_path}")
