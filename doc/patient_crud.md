# Patient CRUD

Implementation in tree-based model (PatientTreeModel)

## Create

1. Calculate Patient_ID = md5(datetime + patient:name)
2. Create directory ./data/patients/Patient_ID
3. Create directory for patient's scans ./data/patients/Patient_ID/scans
4. Create patient's metadata file in ./data/patients/Patient_ID/metadata.json

### Issues / Questions

* Using patient's PESEL number as Patient_ID?


## Read

### Read by Patient_ID

1. Read metadata file ./data/patients/Patient_ID/metadata.json
2. Construct Patient instance from JSON data

### Read all

1. Navigate through all directories in ./data/patients/
2. Every directory name consider as Patient_ID and read from it

## Update

1. Find ./data/patients/Patient_ID directory
2. Open and modify metadata.json

## Delete

### Delete by Patient_ID

1. Find ./data/patients/Patient_ID directory
2. Delete whole content and Patient_ID directory at the end