# Configuration PDL de la STATION SOL — à installer en /etc/default/wfb
#
# Ports radio séparés par sens, pour que les deux directions ne se mélangent
# pas : 0 descendant (drone -> sol), 1 montant (sol -> drone).
#
# Ports UDP vus du drone : son UdpTelemetryDriver écoute 5601 et émet vers
# 5602. wfb_tx lit donc 5602, et wfb_rx livre sur 5601.
WFB_CHANNEL=1
WFB_KEY=gs.key
WFB_TX_RADIO_PORT=1
WFB_TX_UDP_PORT=5601
WFB_RX_RADIO_PORT=0
WFB_RX_UDP_PORT=5602
WFB_STATS_PORT=5699
