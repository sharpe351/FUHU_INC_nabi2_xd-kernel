#ifndef __LINUX_BQ27X00_UPDATE_H__
#define __LINUX_BQ27X00_UPDATE_H__

struct bq27x00_block_data_info{
	int subclass;
	int offset;
	int data;
	int length;
};
	
struct bq27x00_update_access_resource{
	
	struct i2c_client *client;
	struct i2c_client *client_up;
	int update_mode;
	
	int (*open)(struct bq27x00_update_access_resource *, struct i2c_client *);
	int (*read_blockdata)(struct bq27x00_update_access_resource *, struct bq27x00_block_data_info *);
	int (*write_blockdate)(struct bq27x00_update_access_resource *, struct bq27x00_block_data_info *);

	int (*write_operation_configuration)(struct bq27x00_update_access_resource *, u16);
	int (*read_operation_configuration)(struct bq27x00_update_access_resource *);
	
	int (*chip_force_reset)(struct bq27x00_update_access_resource *);
	int (*update_firmware)(struct bq27x00_update_access_resource *);
	int (*check_rom_mode)(struct bq27x00_update_access_resource *);
};

extern struct bq27x00_update_access_resource* get_bq27x00_update_instance( void );

#endif
	
