import { AxiosError } from 'axios';
import { Alert } from 'react-native';
import { ApiErrorResponse, ErrorCode } from './types';

/**
 * UTILITÁRIOS DE TRATAMENTO DE ERROS
 * 
 * Este arquivo contém funções auxiliares para tratamento e exibição de erros
 * de forma consistente em toda a aplicação.
 */

/**
 * Classe de erro personalizada para a API
 */
export class ApiError extends Error {
  public code: ErrorCode;
  public statusCode?: number;
  public details?: any;

  constructor(message: string, code: ErrorCode, statusCode?: number, details?: any) {
    super(message);
    this.name = 'ApiError';
    this.code = code;
    this.statusCode = statusCode;
    this.details = details;
  }
}

/**
 * Extrai informações de erro de uma resposta Axios
 * 
 * @param error - Erro do Axios
 * @returns ApiError formatado
 * 
 * Exemplo de uso:
 * ```typescript
 * try {
 *   await apiClient.get('/endpoint');
 * } catch (error) {
 *   const apiError = handleApiError(error);
 *   console.error(apiError.message);
 * }
 * ```
 */
export const handleApiError = (error: unknown): ApiError => {
  if (error instanceof AxiosError) {
    // Erro de rede (sem resposta do servidor)
    if (!error.response) {
      return new ApiError(
        'Sem conexão com o servidor. Verifique sua conexão com a internet.',
        ErrorCode.NETWORK_ERROR
      );
    }

    // Timeout
    if (error.code === 'ECONNABORTED') {
      return new ApiError(
        'Tempo de requisição excedido. Tente novamente.',
        ErrorCode.TIMEOUT
      );
    }

    const { status, data } = error.response;

    // Tenta extrair mensagem de erro da resposta
    const errorData = data as ApiErrorResponse;
    const message = errorData?.error?.message || error.message || 'Erro desconhecido';

    // Mapeia status HTTP para código de erro
    switch (status) {
      case 401:
        return new ApiError(
          'Não autorizado. Faça login novamente.',
          ErrorCode.UNAUTHORIZED,
          status,
          data
        );
      
      case 403:
        return new ApiError(
          'Acesso negado. Você não tem permissão.',
          ErrorCode.FORBIDDEN,
          status,
          data
        );
      
      case 404:
        return new ApiError(
          'Recurso não encontrado.',
          ErrorCode.NOT_FOUND,
          status,
          data
        );
      
      case 422:
        return new ApiError(
          message || 'Dados inválidos.',
          ErrorCode.VALIDATION_ERROR,
          status,
          data
        );
      
      case 500:
      case 502:
      case 503:
        return new ApiError(
          'Erro no servidor. Tente novamente mais tarde.',
          ErrorCode.SERVER_ERROR,
          status,
          data
        );
      
      default:
        return new ApiError(
          message,
          ErrorCode.UNKNOWN_ERROR,
          status,
          data
        );
    }
  }

  // Erro não é do Axios
  if (error instanceof Error) {
    return new ApiError(error.message, ErrorCode.UNKNOWN_ERROR);
  }

  // Erro desconhecido
  return new ApiError('Erro desconhecido', ErrorCode.UNKNOWN_ERROR);
};

/**
 * Exibe um Alert com a mensagem de erro
 * 
 * @param error - Erro a ser exibido
 * @param customTitle - Título customizado do alert (opcional)
 * 
 * Exemplo de uso:
 * ```typescript
 * try {
 *   await apiClient.get('/endpoint');
 * } catch (error) {
 *   showErrorAlert(error);
 * }
 * ```
 */
export const showErrorAlert = (error: unknown, customTitle?: string): void => {
  const apiError = handleApiError(error);
  
  Alert.alert(
    customTitle || 'Erro',
    apiError.message,
    [{ text: 'OK', style: 'cancel' }]
  );
};

/**
 * Retorna uma mensagem amigável baseada no código de erro
 * 
 * @param code - Código do erro
 * @returns Mensagem amigável
 */
export const getErrorMessage = (code: ErrorCode): string => {
  const messages: Record<ErrorCode, string> = {
    [ErrorCode.NETWORK_ERROR]: 'Verifique sua conexão com a internet',
    [ErrorCode.TIMEOUT]: 'A operação demorou muito tempo',
    [ErrorCode.UNAUTHORIZED]: 'Você precisa estar logado',
    [ErrorCode.FORBIDDEN]: 'Você não tem permissão para esta ação',
    [ErrorCode.NOT_FOUND]: 'O recurso não foi encontrado',
    [ErrorCode.VALIDATION_ERROR]: 'Dados inválidos',
    [ErrorCode.SERVER_ERROR]: 'Erro no servidor',
    [ErrorCode.UNKNOWN_ERROR]: 'Erro desconhecido',
  };

  return messages[code] || messages[ErrorCode.UNKNOWN_ERROR];
};

/**
 * Verifica se o erro é um erro de autenticação
 * 
 * @param error - Erro a verificar
 * @returns true se for erro de autenticação
 */
export const isAuthError = (error: unknown): boolean => {
  const apiError = handleApiError(error);
  return apiError.code === ErrorCode.UNAUTHORIZED;
};

/**
 * Verifica se o erro é um erro de rede
 * 
 * @param error - Erro a verificar
 * @returns true se for erro de rede
 */
export const isNetworkError = (error: unknown): boolean => {
  const apiError = handleApiError(error);
  return apiError.code === ErrorCode.NETWORK_ERROR;
};

/**
 * Loga o erro no console (apenas em desenvolvimento)
 * 
 * @param error - Erro a logar
 * @param context - Contexto adicional (ex: nome da função)
 */
export const logError = (error: unknown, context?: string): void => {
  if (__DEV__) {
    const apiError = handleApiError(error);
    console.error('🔴 API Error:', {
      context,
      code: apiError.code,
      message: apiError.message,
      statusCode: apiError.statusCode,
      details: apiError.details,
    });
  }
};

/**
 * COMO USAR NO COMPONENTE:
 * 
 * import { handleApiError, showErrorAlert, logError } from '@/api/errorHandler';
 * 
 * const handleAction = async () => {
 *   try {
 *     await someApiCall();
 *   } catch (error) {
 *     // Opção 1: Apenas mostrar alert
 *     showErrorAlert(error);
 *     
 *     // Opção 2: Tratar erro manualmente
 *     const apiError = handleApiError(error);
 *     if (apiError.code === ErrorCode.UNAUTHORIZED) {
 *       router.push('/login');
 *     }
 *     
 *     // Opção 3: Logar erro
 *     logError(error, 'handleAction');
 *   }
 * };
 */
