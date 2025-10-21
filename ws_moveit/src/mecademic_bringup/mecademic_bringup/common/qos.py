from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

def qos_default(depth: int = 10) -> QoSProfile:
    """Standard-QoS für stabile Kommunikation."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

def qos_latched(depth: int = 1) -> QoSProfile:
    """Für latched Topics (z. B. Posen, Marker)."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

def qos_sensor_data(depth: int = 5) -> QoSProfile:
    """Für hochfrequente Sensorstreams (z. B. Jogging)."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

__all__ = ["qos_default", "qos_latched", "qos_sensor_data"]
