/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


/****************************************************************************
*
* This file is used for eddystone receiver.
*
****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>

#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "freeRTOS/FreeRTOS.h"
#include "freeRTOS/task.h"


#include "esp_eddystone_protocol.h"
#include "esp_eddystone_api.h"
#include "cJSON.h"
#include "foo.h"


static const char* DEMO_TAG = "EDDYSTONE_DEMO";
int j = 0;
int buffercounter = 10;
uint64_t time1;
uint64_t time2;
uint64_t time_point_bf;
uint64_t time_point_nbf;

cJSON *root,*fmt;

char tx_item[200];


typedef struct Beacons {
	uint64_t UUID;
	int RSSI;
	char *ID;
	int time_stamp;
	bool arrived ;
	bool left ;
} Beacon;

Beacon beacon_arr[50];
Beacon beacon_arr_20[50];
Beacon beacon_arr_30[50];
int top,top_20,top_30=0;

/* declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);


static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,			.own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,	.scan_interval          = 0x50,
    .scan_window            = 0x30,							.scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

// get the time stamp in sec
int64_t xx_time_get_time() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}


static void beacon_stuctures_filtering(int k, Beacon *beacon, int RSSI,	uint64_t dev_addr, char *buffer, int counter) {
	bool unique = true;

	for (int m=0; m<k+1; m++){
		if (dev_addr == beacon[m].UUID){
			unique = false;
			printf("Not unique \n");
			j--;
			break;
		}
	}

	if (unique){
		if ((RSSI>-65) && (strcmp(buffer, "476c6f62616c2d546167")==0 )){
			beacon[k].UUID = dev_addr;
			beacon[k].RSSI = RSSI;
			beacon[k].ID = buffer;
			beacon[k].time_stamp = counter;
			top=k+1;

			printf("'k': %d,  \n", k);
			printf("Detected Beacon ------------------------------------- \n");
			printf("'UUID': '%llx',\n", beacon[k].UUID);
		/*	printf("'RSSI': %d,  \n", beacon[k].RSSI);*/
			printf("'Namespace ID': '%s' \n", beacon[k].ID);
			printf("'Time stamp': %d,  \n", beacon[k].time_stamp);
		}
		else {
			printf("Out of filter \n");
			j--;
		}
	}
}

char *create_monitor(uint64_t mac_addr, bool detected)
{
	char buffer_mac_addr[100];
	char buffer_detection[50];
    char *string = NULL;
  //  cJSON *detection = NULL;

    cJSON *monitor = cJSON_CreateObject();

    if (monitor == NULL)
    {
        goto end;
    }

	sprintf( buffer_mac_addr, "%llx", mac_addr );
    sprintf( buffer_detection, "%d", detected );

    cJSON_AddStringToObject(monitor ,"MAC Addr", buffer_mac_addr);
    cJSON_AddStringToObject(monitor,"room",		"waiting room");
    cJSON_AddStringToObject(monitor ,"detected", buffer_detection);

    printf("JSON PRINT---------------------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%-------------------------------------- \n");
    string = cJSON_Print(monitor);
    if (string == NULL)
    {
        fprintf(stderr, "Failed to print monitor.\n");
    }
    else{
    	//printf("%s \n", string);
        sprintf(tx_item, "beacon: %s.", string);
    	//Send an item
    	UBaseType_t res =  xRingbufferSend(buf_handle, tx_item, sizeof(tx_item), pdMS_TO_TICKS(1000));
    	if (res != pdTRUE) {
    		printf("Failed to send item\n");
    	}
    }

end:
    cJSON_Delete(monitor);
    return string;
}


static void detect_arriving_beacons (){
	bool maybe_just_arrived=false;
	//printf("Detecting the arriving beacons--------------------- \n");

	for (int n=0; n<top; n++){ // for each beacon of arr10
		for (int m=0; m<top_20; m++){// compare with ten seconds before
			if (beacon_arr[n].UUID == beacon_arr_20[m].UUID){
				maybe_just_arrived=false;
				beacon_arr[n].arrived = false;
				break;// beacon is already arrived
			}
			else{
				maybe_just_arrived = true;
			}
		}
		// after checking all the indexes of beacon_arr_20, check with beacon_arr_30
		if (maybe_just_arrived == true){
			for (int x=0; x<top_30; x++){
				if (beacon_arr[n].UUID == beacon_arr_30[x].UUID){
					beacon_arr[n].arrived = false;
					break;//  in the room
				}
				else{
					beacon_arr[n].arrived = true;
				}
			}
			maybe_just_arrived=false;
			if ((beacon_arr[n].arrived) && (beacon_arr[n].UUID != 10) ){ create_monitor(beacon_arr[n].UUID, true);}
		}
	}
}

static void detect_left_beacons (){
	bool maybe_left=false;
	//printf("Detecting the leaving beacons--------------------- \n");
	for (int n=0; n<top_30; n++){ // for each beacon of arr3
		for (int m=0; m<top_20; m++){// compare with ten seconds after
			if (beacon_arr_30[n].UUID == beacon_arr_20[m].UUID){
				beacon_arr_30[n].left = false;
				maybe_left=false;
				break;// beacon is still being detected
			}
			else{
				maybe_left = true;
			}
		}

		if (maybe_left == true){
			for (int x=0; x<top; x++){
				if (beacon_arr_30[n].UUID == beacon_arr[x].UUID){
					beacon_arr_30[n].left = false;
					break;// still in the room
				}
				else{
					beacon_arr_30[n].left = true;
				}
			}
			maybe_left=false;
			if (beacon_arr_30[n].left && (beacon_arr_30[n].UUID != 10) ){ create_monitor(beacon_arr_30[n].UUID, false);}
		}
	}
}

void print_list(){
	buffercounter = buffercounter +1;

	//printf("'Cansu': %d,  \n", val );
//	printf("'U': %d,  \n", buffercounter );


	for (int k=0; k<top; k++){
			printf("The LIST---------------------ARR_10 \n");
			printf("'UUID': '%llx',\n", beacon_arr[k].UUID);
		/*	printf("'RSSI': %d,  \n", beacon_arr[k].RSSI);
			printf("'Namespace ID': '%s' \n", beacon_arr[k].ID);*/
			printf("'Time stamp': %d,  \n", beacon_arr[k].time_stamp);
			printf("'arrived': %d,  \n", beacon_arr[k].arrived);
			printf("'left': %d,  \n", beacon_arr[k].left);
		}
	for (int k=0; k<top_20; k++){
			printf("The LIST---------------------ARR_20 \n");
			printf("'UUID': '%llx',\n", beacon_arr_20[k].UUID);
		/*	printf("'RSSI': %d,  \n", beacon_arr_20[k].RSSI);
			printf("'Namespace ID': '%s' \n", beacon_arr_20[k].ID);*/
			printf("'Time stamp': %d,  \n", beacon_arr_20[k].time_stamp);
			printf("'arrived': %d,  \n", beacon_arr_20[k].arrived);
			printf("'left': %d,  \n", beacon_arr_20[k].left);
		}
	for (int k=0; k<top_30; k++){
		printf("The LIST---------------------ARR_30 \n");
		printf("'UUID': '%llx',\n", beacon_arr_30[k].UUID);
	/*	printf("'RSSI': %d,  \n", beacon_arr_30[k].RSSI);
		printf("'Namespace ID': '%s' \n", beacon_arr_30[k].ID);*/
		printf("'Time stamp': %d,  \n", beacon_arr_30[k].time_stamp);
		printf("'arrived': %d,  \n", beacon_arr_30[k].arrived);
		printf("'left': %d,  \n", beacon_arr_30[k].left);
	}
}

//create a monitor with a list of supported resolutions
//NOTE: Returns a heap allocated string, you are required to free it after use.



static void past_info(Beacon *beacon){
	//printf("Shifting arrays \n");
	for(int i=0; i<top_20; i++){
		beacon_arr_30[i] = beacon_arr_20[i];
		top_30 =top_20;
	}
	for(int j=0; j<top; j++){
		beacon_arr_20[j] = beacon_arr[j];
		top_20=top;
	}
}


int counter = 0;
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
	//printf("esp_gap_cb is called");
	counter++;
	esp_err_t err;
	uint64_t dev_addr;

	uint64_t namespace_id_1;
	uint64_t namespace_id_2;
	char buffer[150];
	char buffer_dev_addr[150];
	int RSSI;

    switch(event)
    {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
			ESP_LOGI(DEMO_TAG, "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT");
            uint32_t duration = 0;
            esp_ble_gap_start_scanning(duration);
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
            if((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(DEMO_TAG,"Scan start failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(DEMO_TAG,"Start scanning...");
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;

            switch(scan_result->scan_rst.search_evt)
            {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                	//	ESP_LOGI(DEMO_TAG,"ESP_GAP_SEARCH_INQ_RES_EVT");
                    esp_eddystone_result_t eddystone_res;
                    memset(&eddystone_res, 0, sizeof(eddystone_res));
                    esp_err_t ret = esp_eddystone_decode(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len, &eddystone_res);

                    time2 = xx_time_get_time();
                    if ((time2-time1)>10000){
                    	j=0;
                    	printf("'TIME DIFFFRENCE':" "%" PRIu64 "\n", time2-time1);
                    	time1 = xx_time_get_time();
                    	if (time2<30000){
                    		past_info(beacon_arr);
                    	}
                    	else{
            				detect_left_beacons();
            				detect_arriving_beacons();
            				print_list();
            			//	create_monitor(0,false);
            				past_info(beacon_arr);
            			}
                    }


                    if (ret) {
                    	time_point_nbf = xx_time_get_time();
                    	if ((time_point_nbf-time_point_bf)>15000 && (time_point_nbf-time_point_bf)<17300){

                    		ESP_LOGI(DEMO_TAG, "--------NO BEACON Found----------");
                    		beacon_stuctures_filtering(0, &beacon_arr, 0, 10, "476c6f62616c2d546167",counter); // assign our namespace id

                    	}

                        // error:The received data is not an eddystone frame packet or a correct eddystone frame packet.
                        // just return
                        return;
                    }
					else {
						// The received adv data is a correct eddystone frame packet.
						// Here, we get the eddystone infomation in eddystone_res, we can use the data in res to do other things.
						// For example, just print them:
						ESP_LOGI(DEMO_TAG, "--------Eddystone Found----------");
						time_point_bf = xx_time_get_time();
					//	any_beacon = true;
						//esp_log_buffer_hex("EDDYSTONE_DEMO: Device address:", scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
						//ESP_LOGI(DEMO_TAG, "RSSI of packet:%d dbm", scan_result->scan_rst.rssi);
						RSSI = scan_result->scan_rst.rssi; //getting RSSI
						//getting  dev address
						dev_addr = ((uint64_t) scan_result->scan_rst.bda[0] << 40)
								| ((uint64_t) scan_result->scan_rst.bda[1] << 32)
								| ((uint64_t) scan_result->scan_rst.bda[2] << 24)
								| ((uint64_t) scan_result->scan_rst.bda[3] << 16)
								| ((uint64_t) scan_result->scan_rst.bda[4] << 8)
								| (scan_result->scan_rst.bda[5]);
						sprintf(buffer_dev_addr, "%llx", dev_addr);
						//getting namespace id
						namespace_id_1 = ((uint64_t) (&eddystone_res)->inform.uid.namespace_id[0]   << 40)
										| ((uint64_t) (&eddystone_res)->inform.uid.namespace_id[1]	<< 32)
										| ((uint64_t) (&eddystone_res)->inform.uid.namespace_id[2]	<< 24)
										| ((uint64_t) (&eddystone_res)->inform.uid.namespace_id[3]	<< 16)
										| ((uint64_t) (&eddystone_res)->inform.uid.namespace_id[4]	<< 8)
										| ((&eddystone_res)->inform.uid.namespace_id[5]);
						namespace_id_2 =((uint64_t) (&eddystone_res)->inform.uid.namespace_id[6]    << 24)
										| ((uint64_t) (&eddystone_res)->inform.uid.namespace_id[7]	<< 16)
										| ((uint64_t) (&eddystone_res)->inform.uid.namespace_id[8]	<< 8)
										| ((uint64_t) (&eddystone_res)->inform.uid.namespace_id[9]);
						sprintf(buffer, "%llx%llx", namespace_id_1, namespace_id_2);
						//esp_eddystone_show_inform(&eddystone_res);
						beacon_stuctures_filtering(j, &beacon_arr, RSSI, dev_addr, &buffer,counter);
						j++;
					}
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:{
            if((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(DEMO_TAG,"Scan stop failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(DEMO_TAG,"Stop scan successfully");
            }
            break;
        }

        default:
            break;
    }
}


void esp_eddystone_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    //appRegister

    esp_err_t status;
	ESP_LOGI(DEMO_TAG,"Register callback");

	/*<! register the scan callback function to the gap module */
	if((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
		ESP_LOGI(DEMO_TAG,"gap register error: %s", esp_err_to_name(status));
		return;
	}
	else{
		printf("The callback event was set successfully");
	}
}


void print_buffer_function(RingbufHandle_t buf_handle)
{

    for (;;) {
    		printf("First buffer read\n");
    		buffer_read();
    		printf("Second buffer read\n");
    		buffer_read();
    		printf("Third buffer read\n");
    		buffer_read();
    	    vTaskDelay(2000 / portTICK_RATE_MS);
    	}
}

void app_main(void)
{
	time1 = xx_time_get_time();
	time_point_nbf = xx_time_get_time();
	int val = buffer_creator();
	ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_eddystone_init();

	xTaskCreate(print_buffer_function, "print_buffer_function", 4 * 1024, buf_handle, 30, NULL);


    /*<! set scan parameters */
    esp_ble_gap_set_scan_params(&ble_scan_params);

}
