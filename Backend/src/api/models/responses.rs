use serde::Serialize;
use utoipa::ToSchema;

#[derive(Serialize, ToSchema)]
pub struct GenericResponse {
    pub message: String,
}

#[derive(Serialize, ToSchema)]
pub struct ListAllResponse<T: Serialize + ToSchema> {
    pub data: Vec<T>,
    pub message: String,
}

#[derive(Serialize, ToSchema)]
pub struct SingleItemResponse<T: Serialize + ToSchema> {
    pub data: T,
    pub message: String,
}
