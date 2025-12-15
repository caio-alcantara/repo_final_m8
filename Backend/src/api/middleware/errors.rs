use actix_web::HttpResponse;
use actix_web::dev::ServiceResponse;
use actix_web::middleware::ErrorHandlerResponse;

use crate::api::models::responses::GenericResponse;

pub fn json_error_middleware<B>(
    res: ServiceResponse<B>,
) -> actix_web::Result<ErrorHandlerResponse<B>> {
    let resp = match res.response().error() {
        Some(err) => HttpResponse::InternalServerError()
            .content_type("application/json")
            .json(GenericResponse {
                message: err.to_string(),
            })
            .map_into_right_body(),
        None => HttpResponse::build(res.status())
            .finish()
            .map_into_right_body(),
    };

    Ok(ErrorHandlerResponse::Response(res.into_response(resp)))
}
