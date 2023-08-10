// Wifi
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

// Google Cloud Platfom KEY API
const char* googleApiKey = "";

// AWS Root CA Certificate
#define AWS_IOT_ENDPOINT ""

const char AWS_ROOT_CA_CERT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)EOF";

const char AWS_PRIVATE_KEY[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----

-----END RSA PRIVATE KEY-----
)KEY";

const char AWS_DEVICE_CERT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)KEY";
