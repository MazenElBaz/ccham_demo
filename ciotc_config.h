/******************************************************************************
 * Copyright 2019 Google
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
// This file contains your configuration used to connect to Cloud IoT Core

// Important!
// TODO(you): Install root certificate to verify tls connection as described
// in https://www.hackster.io/arichetta/add-ssl-certificates-to-mkr1000-93c89d

// Cloud iot details.
const char* project_id = "singular-ray-341422";
const char* location = "us-central1";
const char* registry_id = "ccham-devices-registry";
const char* device_id = "featherS2-test-1";

// To get the private key run (where private-key.pem is the ec private key
// used to create the certificate uploaded to google cloud iot):
// openssl ec -in <private-key.pem> -noout -text
// and copy priv: part.
// The key length should be exactly the same as the key length bellow (32 pairs
// of hex digits). If it's bigger and it starts with "00:" delete the "00:". If
// it's smaller add "00:" to the start. If it's too big or too small something
// is probably wrong with your key.
const char* private_key_str =
	"22:c5:5e:da:2a:77:cd:8f:14:37:23:0c:06:3c:eb:"
    "ad:61:51:b7:29:eb:71:63:d6:ce:d9:ed:29:8c:62:"
    "d8:7e";

// Time (seconds) to expire token += 20 minutes for drift
const int jwt_exp_secs = 3600; // Maximum 24H (3600*24)