# Patient CRUD

## Create

1. Calculate Patient_ID = md5(datetime + patient:name)
2. Create directory ./data/patients/Patient_ID
3. Create patient's metadata file in ./data/patients/Patient_ID/metadata.json

### Issues / Questions

* Using patient's PESEL number as Patient_ID?


## Read

### Read all

1. Navigate through all directories in ./data/patients/ dir
2. Go inside every scanned directory and read metadata.json

## Update

## Delete

### Delete by Patient_ID

1. Find ./data/patients/Patient_ID directory
2. Delete whole content and Patient_ID directory at the end