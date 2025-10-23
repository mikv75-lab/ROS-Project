# ros/common/qos.py
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

def qos_default(depth: int = 10) -> QoSProfile:
    """Standard QoS-Profil für zuverlässige Kommunikation mit einem Tiefe-Puffer."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

def qos_latched(depth: int = 1) -> QoSProfile:
    """QoS-Profil für latched Topics (z. B. RViz-Previews, Posen/Marker)."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

def qos_sensor_data(depth: int = 5) -> QoSProfile:
    """Für hochfrequente Datenströme wie Jogging oder Sensor-Daten."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

def qos_best_effort(depth: int = 10) -> QoSProfile:
    """Für weniger kritische Datenströme (Best Effort, weniger zuverlässig)."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
